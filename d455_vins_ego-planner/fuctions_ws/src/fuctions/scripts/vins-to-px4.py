#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys, math
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class FastVinsMavrosBridge:
    def __init__(self):
        rospy.init_node("vins_px4_bridge", anonymous=False)
        self.rate = rospy.Rate(30)
        
        self.v_pose, self.p_pose = PoseStamped(), PoseStamped()
        self.v_att, self.p_att = [0.0]*3, [0.0]*3
        self.has_vins = False
        
        # --- 核心补偿参数 (基于你的数据微调) ---
        # 你的 VINS 静态时 Roll 是 -92.9，所以我们需要把它加回到 0 附近
        self.vins_roll_static_offset = -92.9  
        # Yaw 的偏移量，根据你的数据 174.6 - 97.2 ≈ 77
        self.yaw_offset = 0

        self.v_pose.header.frame_id = "map"
        rospy.Subscriber("/vins_estimator/odometry", Odometry, self.vins_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.px4_cb)
        self.pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)

    def vins_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # 1. 位置保持你之前调好的 (y, -x, z)
        self.v_pose.pose.position = Point(pos.y, -pos.x, pos.z)
        
        # 2. 姿态映射逻辑修复：
        q_raw = [ori.x, ori.y, ori.z, ori.w]
        (r, p, y) = euler_from_quaternion(q_raw)
        
        # 根据你的测试：抬头时 PX4_Pitch 变，VINS_Roll 变 -> 说明轴交换了
        # 我们重新映射欧拉角：
        # r_new (PX4 Roll)  应该受 VINS Pitch 影响
        # p_new (PX4 Pitch) 应该受 VINS Roll 影响
        
        r_new = p  # VINS 的俯仰变为 PX4 的横滚
        
        # 关键修正：将 VINS 的 Roll 补偿后赋给 Pitch
        # 因为 VINS Roll 静态是 -92，我们要让它回到 0：(r - (-92))
        # 且抬头时 VINS Roll 是正，PX4 Pitch 是负，所以前面加个负号
        p_new = -(r - math.radians(self.vins_roll_static_offset))
        
        # 航向角对齐
        y_new = y + math.radians(self.yaw_offset)

        # 3. 合成四元数
        q_new = quaternion_from_euler(r_new, p_new, y_new)
        self.v_pose.pose.orientation = Quaternion(*q_new)

        # 更新显示数据
        self.v_att = [math.degrees(r_new), math.degrees(p_new), math.degrees(y_new)]
        self.has_vins = True

    def px4_cb(self, msg):
        self.p_pose = msg
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.p_att = [math.degrees(a) for a in euler_from_quaternion(q)]

    def start(self):
        while not rospy.is_shutdown():
            if not self.has_vins:
                sys.stdout.write("\033[K\033[31m⚠️ 等待 VINS 数据输入...\033[0m\r")
            else:
                self.v_pose.header.stamp = rospy.Time.now()
                self.pub.publish(self.v_pose)

                vp, pp = self.v_pose.pose.position, self.p_pose.pose.position
                dashboard = (
                    f"\033[K\033[32m[ VINS -> PX4 Bridge OK ]\033[0m\n"
                    f"\033[K       VINS (Fixed)            PX4 (Local)\n"
                    f"\033[Kx      {vp.x:10.4f}\t\t{pp.x:10.4f}\n"
                    f"\033[Ky      {vp.y:10.4f}\t\t{pp.y:10.4f}\n"
                    f"\033[Kz      {vp.z:10.4f}\t\t{pp.z:10.4f}\n"
                    f"\033[Kroll   {self.v_att[0]:10.4f}\t\t{self.p_att[0]:10.4f}\n"
                    f"\033[Kpitch  {self.v_att[1]:10.4f}\t\t{self.p_att[1]:10.4f}\n"
                    f"\033[Kyaw    {self.v_att[2]:10.4f}\t\t{self.p_att[2]:10.4f}\n"
                )
                sys.stdout.write(dashboard)
                sys.stdout.flush()
                sys.stdout.write("\033[8A")
            self.rate.sleep()
        sys.stdout.write("\033[8B\n")

if __name__ == "__main__":
    try:
        FastVinsMavrosBridge().start()
    except rospy.ROSInterruptException:
        pass
