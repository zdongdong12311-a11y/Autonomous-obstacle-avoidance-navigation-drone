#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np

class VinsToMavros:
    def __init__(self):
        rospy.init_node('vins_to_px4_bridge')

        # 目标发布频率（PX4 EKF2 推荐 30Hz）
        self.publish_rate = rospy.get_param('~publish_rate', 250)
        self.last_pub_time = rospy.Time(0)
        self.min_interval = rospy.Duration(1.0 / self.publish_rate)

        # 坐标系转换参数
        self.swap_xy = rospy.get_param('~swap_xy', True)  # 交换 X 和 Y
        self.swap_xz = rospy.get_param('~swap_xz', False)  # 交换 X 和 Z
        self.negate_x = rospy.get_param('~negate_x', False)  # X 取反
        self.negate_y = rospy.get_param('~negate_y', False)  # Y 取反
        self.negate_z = rospy.get_param('~negate_z', False)  # Z 取反
        
        # 发布到 MAVROS 视觉位姿话题
        self.pub = rospy.Publisher(
            '/mavros/vision_pose/pose',
            PoseStamped,
            queue_size=10
        )

        # TF 广播器（EKF2 需要 map -> base_link 的 TF）
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 订阅 VINS 里程计
        rospy.Subscriber(
            '/vins_estimator/odometry',
            Odometry,
            self.odom_cb,
            queue_size=10
        )

        rospy.loginfo("VINS -> PX4 桥接节点已启动")
        rospy.loginfo("坐标系转换: swap_xy=%s", self.swap_xy)

    def transform_position(self, pos):
        """转换位置坐标"""
        x = pos.x
        y = pos.y
        z = pos.z
        
        if self.swap_xy:
            # 交换 X 和 Y
            x, y = y, x
            rospy.logdebug("交换 XY: (%f, %f, %f) -> (%f, %f, %f)", 
                          pos.x, pos.y, pos.z, x, y, z)
        
        if self.swap_xz:
            # 交换 X 和 Z
            x, z = z, x
        
        if self.negate_x:
            x = -x
        if self.negate_y:
            y = -y
        if self.negate_z:
            z = -z
        
        return Point(x, y, z)
    
    def transform_orientation(self, quat):
        """转换姿态（如果需要旋转坐标系）"""
        # 如果交换了 XY，需要旋转四元数
        if self.swap_xy:
            # 绕 Z 轴旋转 90 度（或 -90 度）
            # 这里根据实际情况调整
            from scipy.spatial.transform import Rotation as R
            
            # 原始四元数
            q_orig = [quat.x, quat.y, quat.z, quat.w]
            
            # 创建旋转（根据实际情况选择）
            # 选项1: 绕 Z 轴旋转 90 度
            r_rot = R.from_euler('z', 90, degrees=True)
            # 选项2: 绕 Z 轴旋转 -90 度
            # r_rot = R.from_euler('z', -90, degrees=True)
            
            # 应用旋转
            q_rot = r_rot.as_quat()
            
            # 四元数乘法
            from scipy.spatial.transform import Rotation as R
            r_orig = R.from_quat(q_orig)
            r_new = r_rot * r_orig
            q_new = r_new.as_quat()
            
            return Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])
        
        # 如果不交换坐标，直接返回
        return Quaternion(quat.x, quat.y, quat.z, quat.w)

    def odom_cb(self, msg):
        now = rospy.Time.now()

        # 频率限制：丢弃过于频繁的消息
        if (now - self.last_pub_time) < self.min_interval:
            return
        self.last_pub_time = now

        # 使用原始时间戳
        stamp = msg.header.stamp

        # 构建 PoseStamped 消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        # 转换位置
        pose_msg.pose.position = self.transform_position(msg.pose.pose.position)
        
        # 转换姿态
        pose_msg.pose.orientation = self.transform_orientation(msg.pose.pose.orientation)

        # 发布视觉位姿
        self.pub.publish(pose_msg)

        # 广播 TF: map -> base_link
        pos = pose_msg.pose.position
        ori = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            stamp,
            "base_link",
            "map"
        )
        
        # 可选：打印调试信息
        if rospy.get_param('~debug', False):
            rospy.loginfo("发布位置: x=%.3f, y=%.3f, z=%.3f", 
                         pos.x, pos.y, pos.z)

if __name__ == '__main__':
    try:
        VinsToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
