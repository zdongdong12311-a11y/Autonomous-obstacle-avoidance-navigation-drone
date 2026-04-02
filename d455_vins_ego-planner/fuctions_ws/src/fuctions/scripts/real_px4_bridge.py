#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import State

class RealPX4Bridge:
    def __init__(self):
        rospy.init_node('real_px4_bridge')
        
        # 1. 发布给飞控的局部位置设定点
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # 2. 接收 Ego-Planner 发来的避障轨迹指令
        rospy.Subscriber('/position_cmd', PositionCommand, self.cmd_cb)
        
        # 3. 监听飞控当前模式 (极其重要的安全机制)
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.current_mode = ""

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        
        # 实机起飞安全悬停高度 (1.2米)
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 1.2
        self.target_pose.pose.orientation.w = 1.0

        # 以 20Hz 的超高频率向飞控发送指令 (飞控 OFFBOARD 模式的硬性要求)
        rospy.Timer(rospy.Duration(0.05), self.timer_cb)
        rospy.loginfo("🛡️ 实机脊髓已启动！悬停高度 1.2m。等待遥控器切入 OFFBOARD 模式...")

    def state_cb(self, msg):
        self.current_mode = msg.mode

    def cmd_cb(self, msg):
        # ⚠️ 防触地保护：如果算法发疯让飞机钻地 (Z < 0.5m)，强行拉升到 1.2 米！
        if msg.position.z < 0.5:
            self.target_pose.pose.position.z = 1.2
            return
            
        self.target_pose.pose.position.x = msg.position.x
        self.target_pose.pose.position.y = msg.position.y
        self.target_pose.pose.position.z = msg.position.z

    def timer_cb(self, event):
        # ⚠️ 遥控器夺权保护：只有当遥控器真实切入了 OFFBOARD 模式，脚本才往外发坐标！
        # 如果你紧急拨动开关切回 Position，脚本立刻闭嘴，绝不跟遥控器抢方向盘！
        if self.current_mode == "OFFBOARD":
            self.target_pose.header.stamp = rospy.Time.now()
            self.pub.publish(self.target_pose)

if __name__ == '__main__':
    try:
        RealPX4Bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass