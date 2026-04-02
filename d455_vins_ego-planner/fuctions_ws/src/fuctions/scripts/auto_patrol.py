#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped

class AutoPatrol:
    def __init__(self):
        rospy.init_node('auto_patrol_node')
        # 伪装成 RViz 的点击，向翻译官发送目标
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.sleep(2.0)

        # 巡航航点 [X, Y, Z] (单位：米)
        # ⚠️ 实飞前务必根据场地大小修改这些点，不要让飞机撞墙！
        self.waypoints = [
            [ 3.0,  0.0, 1.2], # 点 1
            [ 3.0,  3.0, 1.2], # 点 2
            [ 0.0,  3.0, 1.2], # 点 3
            [ 0.0,  0.0, 1.2]  # 回到起点
        ]

    def start_patrol(self):
        rospy.loginfo("🚨 自动巡逻任务开始！请手指放在遥控器上，随时准备切回手动！")
        
        for i, wp in enumerate(self.waypoints):
            rospy.loginfo(f"🚀 正在飞向航点 {i+1}: X={wp[0]}, Y={wp[1]}, Z={wp[2]}")
            
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = wp[0]
            goal.pose.position.y = wp[1]
            goal.pose.position.z = wp[2]
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            
            # 给飞机 15 秒的时间避障并飞过去
            rospy.sleep(15.0) 

        rospy.loginfo("✅ 巡航完毕，无人机已返回起点！")

if __name__ == '__main__':
    try:
        AutoPatrol().start_patrol()
    except rospy.ROSInterruptException:
        pass