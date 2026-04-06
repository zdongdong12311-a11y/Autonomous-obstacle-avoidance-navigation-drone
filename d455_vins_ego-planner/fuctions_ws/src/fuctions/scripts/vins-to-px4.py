#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class VinsMavrosMerged:
    def __init__(self):
        rospy.init_node("vins_to_px4_merged", anonymous=False)
        self.pi = math.pi
        self.estimated_odom_rec_flag = False

        # --- 1. 原封不动继承 VINS 坐标转换参数 ---
        self.publish_rate = rospy.get_param('~publish_rate', 30)
        self.swap_xy = rospy.get_param('~swap_xy', True)
        self.swap_xz = rospy.get_param('~swap_xz', False)
        self.negate_x = rospy.get_param('~negate_x', False)
        self.negate_y = rospy.get_param('~negate_y', True)
        self.negate_z = rospy.get_param('~negate_z', False)
        
        self.apply_yaw_rotation = rospy.get_param('~apply_yaw_rotation', True)
        self.yaw_rotation_deg = rospy.get_param('~yaw_rotation_deg', 90.0)

        # --- 2. 姿态与坐标数据容器 ---
        self.estimated_attitude = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self.px4_attitude = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}

        self.estimated_pose = PoseStamped()
        self.estimated_pose.header.frame_id = "map"
        self.px4_pose = PoseStamped()

        # --- 3. 订阅与发布 ---
        self.px4_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.px4_pose_cb)
        self.odom_sub = rospy.Subscriber("/vins_estimator/odometry", Odometry, self.odom_cb)
        self.vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("VINS to PX4 Merged Node Initialized.")

    # ================= 核心数学转换逻辑 (保留不改) =================
    def transform_position(self, pos):
        x, y, z = pos.x, pos.y, pos.z
        if self.swap_xy: x, y = y, x
        if self.swap_xz: x, z = z, x
        if self.negate_x: x = -x
        if self.negate_y: y = -y
        if self.negate_z: z = -z
        return Point(x, y, z)
    
    def quat_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return [
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ]

    def transform_orientation(self, quat):
        if not self.apply_yaw_rotation:
            return Quaternion(quat.x, quat.y, quat.z, quat.w)
        
        theta = np.radians(self.yaw_rotation_deg)
        half_theta = theta / 2.0
        rot_q = [np.cos(half_theta), 0, 0, np.sin(half_theta)] 
        orig_q = [quat.w, quat.x, quat.y, quat.z] 
        
        q_new = self.quat_multiply(rot_q, orig_q)
        return Quaternion(q_new[1], q_new[2], q_new[3], q_new[0]) 

    # ================= 回调处理 =================
    def odom_cb(self, msg):
        """里程计回调：执行转换并存入容器"""
        # 1. 执行严格的位置和姿态转换
        t_pos = self.transform_position(msg.pose.pose.position)
        t_ori = self.transform_orientation(msg.pose.pose.orientation)

        self.estimated_pose.pose.position = t_pos
        self.estimated_pose.pose.orientation = t_ori

        # 2. 四元数转欧拉角用于看板显示
        quat = (t_ori.x, t_ori.y, t_ori.z, t_ori.w)
        roll, pitch, yaw = euler_from_quaternion(quat)

        self.estimated_attitude["pitch"] = pitch * 180.0 / self.pi
        self.estimated_attitude["roll"] = roll * 180.0 / self.pi
        self.estimated_attitude["yaw"] = yaw * 180.0 / self.pi

        self.estimated_odom_rec_flag = True

    def px4_pose_cb(self, msg):
        """PX4位置回调：提取位姿和姿态"""
        self.px4_pose.pose = msg.pose
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quat)

        self.px4_attitude["pitch"] = pitch * 180.0 / self.pi
        self.px4_attitude["roll"] = roll * 180.0 / self.pi
        self.px4_attitude["yaw"] = yaw * 180.0 / self.pi

    # ================= 主循环与整齐看板 =================
    def start(self):
        LINES_TO_OVERWRITE = 8
        while not rospy.is_shutdown():
            if not self.estimated_odom_rec_flag:
                sys.stdout.write("\033[K\033[31mWaiting for VINS odometry...\033[0m\n")
                sys.stdout.flush()
            else:
                # 1. 发布转换后的位姿给 PX4
                stamp = rospy.Time.now()
                self.estimated_pose.header.stamp = stamp
                self.vision_pose_pub.publish(self.estimated_pose)

                # 2. 广播 TF 方便 RViz 观看
                pos = self.estimated_pose.pose.position
                ori = self.estimated_pose.pose.orientation
                self.tf_broadcaster.sendTransform(
                    (pos.x, pos.y, pos.z),
                    (ori.x, ori.y, ori.z, ori.w),
                    stamp, "base_link", "map"
                )

                # 3. 极其整齐的一列列看板打印
                lines = []
                lines.append("\033[K\033[32m[ VINS -> PX4 Bridge OK ]\033[0m")
                lines.append("\033[K       VinsPose(Transformed)      px4Pose")
                lines.append(f"\033[Kx      {pos.x:10.4f}\t\t{self.px4_pose.pose.position.x:10.4f}")
                lines.append(f"\033[Ky      {pos.y:10.4f}\t\t{self.px4_pose.pose.position.y:10.4f}")
                lines.append(f"\033[Kz      {pos.z:10.4f}\t\t{self.px4_pose.pose.position.z:10.4f}")
                lines.append(f"\033[Kpitch  {self.estimated_attitude['pitch']:10.4f}\t\t{self.px4_attitude['pitch']:10.4f}")
                lines.append(f"\033[Kroll   {self.estimated_attitude['roll']:10.4f}\t\t{self.px4_attitude['roll']:10.4f}")
                lines.append(f"\033[Kyaw    {self.estimated_attitude['yaw']:10.4f}\t\t{self.px4_attitude['yaw']:10.4f}")

                output = "\n".join(lines)
                sys.stdout.write(output + "\n")
                sys.stdout.flush()

                # 光标上移，完美覆盖不闪烁
                sys.stdout.write(f"\033[{LINES_TO_OVERWRITE}A")
                sys.stdout.flush()

            self.rate.sleep()

        sys.stdout.write(f"\033[{LINES_TO_OVERWRITE}B\n")
        sys.stdout.flush()

def main():
    node = VinsMavrosMerged()
    node.start()

if __name__ == "__main__":
    main()
