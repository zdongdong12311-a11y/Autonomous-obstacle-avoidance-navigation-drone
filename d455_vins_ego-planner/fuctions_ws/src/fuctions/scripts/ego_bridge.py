#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler

class RC_Offboard_Bridge:
    def __init__(self):
        rospy.init_node('rc_offboard_bridge')
        
        # 1. 发布给飞控的控制坐标 (心跳包)
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # 2. 监听飞机真实坐标 (用于影子跟随)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_cb)
        
        # 3. 接收 Ego-Planner 大脑的避障轨迹
        rospy.Subscriber('planning/pos_cmd', PositionCommand, self.cmd_cb)
        
        # 4. 监听遥控器当前切的模式
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        
        self.current_mode = ""
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"

        # 以 30Hz 频率疯狂发送心跳包 (PX4 允许切入 OFFBOARD 的硬性前提)
        rospy.Timer(rospy.Duration(0.033), self.timer_cb)
        
        rospy.loginfo("✅ 遥控器实飞桥接脚本已启动！")
        rospy.loginfo("👉 流程: 1.遥控器起飞 -> 2.悬停后切 OFFBOARD -> 3.接收算法避障")

    def state_cb(self, msg):
        self.current_mode = msg.mode

    def local_pose_cb(self, msg):
        # 🔥 核心：影子跟随逻辑 (Shadowing)
        # 只要你还在用遥控器飞 (没切 OFFBOARD)，我就把目标点实时改成飞机的当前位置。
        # 这样当你拨下 OFFBOARD 开关的那一刻，目标点就是飞机自己，实现 0 顿挫平滑悬停！
        if self.current_mode != "OFFBOARD":
            self.target_pose.pose.position.x = msg.pose.position.x
            self.target_pose.pose.position.y = msg.pose.position.y
            self.target_pose.pose.position.z = msg.pose.position.z
            self.target_pose.pose.orientation = msg.pose.orientation

    def cmd_cb(self, msg):
        # 🧠 只有当你切入了 OFFBOARD 模式，脚本才会听 Ego-Planner 的话！
        if self.current_mode == "OFFBOARD":
            
            # ⚠️ 防触地保护：如果算法发疯让飞机钻地 (Z < 0.3m)，强行托底在 1.0 米！
            if msg.position.z < 0.3:
                self.target_pose.pose.position.z = 1.0 
            else:
                self.target_pose.pose.position.z = msg.position.z
                
            self.target_pose.pose.position.x = msg.position.x
            self.target_pose.pose.position.y = msg.position.y
            
            # 👁️ 偏航角转向：把算法的 Yaw 转成四元数发给飞控，让相机永远看着前方避障！
            q = quaternion_from_euler(0, 0, msg.yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]

    def timer_cb(self, event):
        # 无条件持续发送心跳包 (飞控在其他模式会自动忽略，但切 OFFBOARD 时需要这个凭证)
        self.target_pose.header.stamp = rospy.Time.now()
        self.pub.publish(self.target_pose)

if __name__ == '__main__':
    try:
        RC_Offboard_Bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
