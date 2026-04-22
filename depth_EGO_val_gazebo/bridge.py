#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

class PX4Bridge:
    def __init__(self):
        rospy.init_node('px4_ego_bridge')
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/position_cmd', PositionCommand, self.cmd_cb)

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        
        # 默认悬停点：正上方 2.0 米
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0
        self.target_pose.pose.orientation.w = 1.0

        rospy.Timer(rospy.Duration(0.05), self.timer_cb)
        rospy.loginfo("脊髓已启动！强制起飞高度已锁定为 2.0 米。等待算法接管...")

    def cmd_cb(self, msg):
        # 完美跟随 XY 避障轨迹，但高度强制托底在 2.0 米，绝对不掉下来！
        self.target_pose.pose.position.x = msg.position.x
        self.target_pose.pose.position.y = msg.position.y
        self.target_pose.pose.position.z = max(msg.position.z, 2.0)

    def timer_cb(self, event):
        self.target_pose.header.stamp = rospy.Time.now()
        self.pub.publish(self.target_pose)

if __name__ == '__main__':
    PX4Bridge()
    rospy.spin()