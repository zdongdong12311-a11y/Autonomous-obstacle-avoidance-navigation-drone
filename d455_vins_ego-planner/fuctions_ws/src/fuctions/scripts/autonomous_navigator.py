#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
自主导航避障系统 - Autonomous Navigation & Obstacle Avoidance
================================================================================
基于 Ego-Planner 避障 + OctoMap 建图的高扩展性导航系统

核心功能:
- 多模式状态机 (待机/起飞/悬停/航点/巡逻/返航/紧急)
- Ego-Planner 全局避障规划
- 影子跟随模式 (遥控器切换无顿挫)
- 安全保护机制

作者: Drone Project
平台: OrangePi 5 Max + D455 + PX4
================================================================================
"""

import rospy
import math
import time
from enum import Enum

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# ================================================================================
# 配置参数
# ================================================================================
class NavConfig:
    """导航配置参数"""

    # ========== 安全参数 ==========
    MIN_ALTITUDE = 0.5       # 最低飞行高度 (m)
    DEFAULT_ALTITUDE = 1.2  # 默认悬停高度 (m)
    TAKEOFF_ALTITUDE = 1.5  # 起飞高度 (m)

    # ========== 航点参数 ==========
    ARRIVAL_THRESHOLD = 0.3   # 到达判定阈值 (m)
    WAYPOINT_TIMEOUT = 30.0    # 航点超时 (s)

    # ========== 巡逻参数 ==========
    PATROL_DWELL_TIME = 2.0  # 航点停留时间 (s)


# ================================================================================
# 导航模式
# ================================================================================
class NavMode(Enum):
    """导航模式"""
    IDLE = "idle"              # 待机 (不控制)
    SHADOW = "shadow"          # 影子跟随 (遥控器控制)
    TAKEOFF = "takeoff"        # 起飞
    HOLD = "hold"              # 悬停 (当前位置)
    WAYPOINT = "waypoint"     # 航点飞行
    PATROL = "patrol"          # 巡逻
    TRACK = "track"            # 目标追踪 (Ego-Planner 控制)
    RETURN = "return"          # 返航
    LAND = "land"              # 降落
    EMERGENCY = "emergency"    # 紧急


class MavMode(Enum):
    """飞控模式"""
    POSITION = "Position"
    OFFBOARD = "OFFBOARD"


# ================================================================================
# 核心导航器
# ================================================================================
class Navigator:
    """
    自主导航核心类

    功能:
    - 影子跟随 (遥控器切换无顿挫)
    - Ego-Planner 避障融合
    - 航点管理
    - 安全保护
    """

    def __init__(self):
        rospy.init_node('autonomous_navigator')

        # ========== 状态 ==========
        self.mode = NavMode.IDLE
        self.mav_mode = ""
        self.is_armed = False

        # ========== 位置数据 ==========
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.has_vins = False

        # ========== Ego-Planner 数据 ==========
        self.planner_cmd = None

        # ========== 电池 ==========
        self.battery_percentage = 1.0

        # ========== 航点数据 ==========
        self.waypoints = []
        self.patrol_index = 0
        self.current_waypoint = None

        # ========== 初始化 ROS ==========
        self._init_publishers()
        self._init_subscribers()

        # ========== 启动控制循环 ==========
        rospy.Timer(rospy.Duration(0.033), self._timer_callback)

        rospy.loginfo("=" * 50)
        rospy.loginfo("自主导航系统已启动!")
        rospy.loginfo("  模式: SHADOW (遥控器控制)")
        rospy.loginfo("  最低高度: {}m".format(NavConfig.MIN_ALTITUDE))
        rospy.loginfo("  默认高度: {}m".format(NavConfig.DEFAULT_ALTITUDE))
        rospy.loginfo("=" * 50)

    def _init_publishers(self):
        """初始化发布者"""
        # 飞控设定点
        self.pub_setpoint = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )

    def _init_subscribers(self):
        """初始化订阅者"""
        # 飞机位置
        rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self._pose_callback
        )

        # 飞控状态
        rospy.Subscriber(
            '/mavros/state',
            State,
            self._state_callback
        )

        # Ego-Planner 避障指令
        rospy.Subscriber(
            '/planning/pos_cmd',
            PositionCommand,
            self._planner_callback
        )

        # VINS 定位
        rospy.Subscriber(
            '/vins_estimator/odometry',
            Odometry,
            self._vins_callback
        )

        # 电池状态
        rospy.Subscriber(
            '/mavros/battery',
            BatteryState,
            self._battery_callback
        )

    # ============================================================================
    # 回调函数
    # ============================================================================
    def _pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg

    def _state_callback(self, msg):
        """飞控状态回调"""
        self.mav_mode = msg.mode
        self.is_armed = msg.armed

    def _planner_callback(self, msg):
        """Ego-Planner 避障指令回调"""
        self.planner_cmd = msg

        # Ego-Planner 控制模式下自动更新目标点
        if self.mode == NavMode.TRACK and self.mav_mode == "OFFBOARD":
            self._apply_planner_cmd(msg)

    def _vins_callback(self, msg):
        """VINS 定位回调"""
        self.has_vins = True

    def _battery_callback(self, msg):
        """电池回调"""
        self.battery_percentage = msg.percentage

    def _timer_callback(self, event):
        """控制循环 (30Hz)"""
        try:
            if self.mav_mode == "OFFBOARD":
                self._publish_setpoint()
                self._safety_check()
        except Exception as e:
            rospy.logerr("[控制循环错误] {}".format(e))

    # ============================================================================
    # 控制发布
    # ============================================================================
    def _publish_setpoint(self):
        """发布设定点到飞控"""
        self.target_pose.header.stamp = rospy.Time.now()
        self.pub_setpoint.publish(self.target_pose)

    def _apply_planner_cmd(self, cmd):
        """应用 Ego-Planner 避障指令"""
        # 防触地保护
        z = cmd.position.z
        if z < NavConfig.MIN_ALTITUDE:
            z = NavConfig.DEFAULT_ALTITUDE

        self.target_pose.pose.position.x = cmd.position.x
        self.target_pose.pose.position.y = cmd.position.y
        self.target_pose.pose.position.z = z

        # 偏航角
        q = quaternion_from_euler(0, 0, cmd.yaw)
        self.target_pose.pose.orientation = Quaternion(*q)

    # ============================================================================
    # 安全检查
    # ============================================================================
    def _safety_check(self):
        """安全检查"""
        # 高度保护
        cur_z = self.current_pose.pose.position.z
        if cur_z < NavConfig.MIN_ALTITUDE:
            rospy.logwarn("[高度过低] 强制拉升!")
            self.target_pose.pose.position.z = NavConfig.DEFAULT_ALTITUDE

        # 电池保护
        if self.battery_percentage < 0.15:
            rospy.logwarn("[电池低] 准备返航!")
            self.mode = NavMode.RETURN
            self.go_home()

    # ============================================================================
    # 影子跟随 (核心)
    # ============================================================================
    def _shadow_follow(self):
        """
        影子跟随逻辑

        当遥控器控制时，实时跟随飞机位置
        切换 OFFBOARD 时实现零顿挫
        """
        self.target_pose.pose.position = self.current_pose.pose.position
        self.target_pose.pose.orientation = self.current_pose.pose.orientation

    # ============================================================================
    # 导航命令
    # ============================================================================
    def set_mode(self, mode):
        """设置导航模式"""
        if mode != self.mode:
            rospy.loginfo("[模式切换] {} -> {}".format(self.mode.value, mode.value))
        self.mode = mode

    def takeoff(self, altitude=None):
        """
        起飞

        Args:
            altitude: 目标高度 (None=默认高度)
        """
        if altitude is None:
            altitude = NavConfig.TAKEOFF_ALTITUDE

        altitude = max(altitude, NavConfig.DEFAULT_ALTITUDE)

        rospy.loginfo("[起飞] 目标高度: {}m".format(altitude))

        # 如果不在 OFFBOARD，先影子跟随
        if self.mav_mode != "OFFBOARD":
            self._shadow_follow()

        # 设置目标高度
        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = altitude
        self.target_pose.pose.orientation.w = 1.0

        self.set_mode(NavMode.TAKEOFF)

        # 等待到达
        self._wait_altitude(altitude)

        rospy.loginfo("[起飞] 完成!")
        self.set_mode(NavMode.HOLD)

    def land(self):
        """降落"""
        rospy.loginfo("[降落] 开始降落...")

        self.set_mode(NavMode.LAND)

        cur_z = self.current_pose.pose.position.z
        step = 0.05

        while cur_z > 0.3 and not rospy.is_shutdown():
            cur_z -= step
            self.target_pose.pose.position.z = max(cur_z, 0.2)
            rospy.sleep(0.1)

        rospy.loginfo("[降落] 完成!")
        self.set_mode(NavMode.IDLE)

    def hold(self):
        """悬停"""
        rospy.loginfo("[悬停] 当前位置悬停")

        self.target_pose.pose.position = self.current_pose.pose.position
        self.target_pose.pose.orientation = self.current_pose.pose.orientation

        self.set_mode(NavMode.HOLD)

    def go_to(self, x, y, z=None, yaw=0.0):
        """
        飞向指定位置

        Args:
            x, y, z: 目标位置
            yaw: 偏航角 (弧度)
        """
        if z is None:
            z = NavConfig.DEFAULT_ALTITUDE
        z = max(z, NavConfig.DEFAULT_ALTITUDE)

        rospy.loginfo("[飞向] 目标: ({}, {}, {})".format(x, y, z))

        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z

        q = quaternion_from_euler(0, 0, yaw)
        self.target_pose.pose.orientation = Quaternion(*q)

        self.set_mode(NavMode.WAYPOINT)
        self.current_waypoint = (x, y, z)

        # 等待到达
        self._wait_position(x, y, z)

        rospy.loginfo("[飞向] 到达!")
        self.set_mode(NavMode.HOLD)

    def go_home(self, altitude=None):
        """返航"""
        if altitude is None:
            altitude = NavConfig.DEFAULT_ALTITUDE

        rospy.loginfo("[返航] 返航点: (0, 0, {})".format(altitude))

        self.set_mode(NavMode.RETURN)
        self.go_to(0.0, 0.0, altitude)
        self.hold()

    def stop(self):
        """停止控制"""
        rospy.loginfo("[停止] 切换到影子跟随模式")

        self.set_mode(NavMode.SHADOW)
        self._shadow_follow()

    def enable_tracking(self):
        """启用 Ego-Planner 追踪"""
        rospy.loginfo("[追踪] 启用 Ego-Planner 避障追踪")

        self.set_mode(NavMode.TRACK)

        if self.planner_cmd:
            self._apply_planner_cmd(self.planner_cmd)

    # ============================================================================
    # 巡线功能
    # ============================================================================
    def set_waypoints(self, waypoints):
        """
        设置航点列表

        Args:
            waypoints: [[x, y, z], ...]
        """
        self.waypoints = waypoints
        rospy.loginfo("[航点] 已设置 {} 个航点".format(len(waypoints)))

    def start_patrol(self, loop=True, dwell_time=None):
        """
        开始巡逻

        Args:
            loop: 是否循环
            dwell_time: 航点停留时间 (s)
        """
        if not self.waypoints:
            rospy.logwarn("[巡逻] 没有设置航点!")
            return

        if dwell_time is None:
            dwell_time = NavConfig.PATROL_DWELL_TIME

        rospy.loginfo("[巡逻] 开始巡逻 {} 个航点".format(len(self.waypoints)))

        self.set_mode(NavMode.PATROL)

        while not rospy.is_shutdown():
            if self.mode != NavMode.PATROL:
                break

            # 获取当前航点
            wp = self.waypoints[self.patrol_index]

            rospy.loginfo("[巡逻] 航点 {}/{}: {}".format(
                self.patrol_index + 1,
                len(self.waypoints),
                wp
            ))

            # 飞向航点
            x, y, z = wp[0], wp[1], wp[2] if len(wp) > 2 else NavConfig.DEFAULT_ALTITUDE
            self.go_to(x, y, z)

            # 停留
            rospy.loginfo("[巡逻] 停留 {}s".format(dwell_time))
            rospy.sleep(dwell_time)

            # 下一个航点
            self.patrol_index = (self.patrol_index + 1) % len(self.waypoints)

            if not loop and self.patrol_index == 0:
                break

        rospy.loginfo("[巡逻] 完成!")
        self.set_mode(NavMode.HOLD)

    def stop_patrol(self):
        """停止巡逻"""
        rospy.loginfo("[巡逻] 停止")
        self.set_mode(NavMode.HOLD)

    # ============================================================================
    # 等待辅助
    # ============================================================================
    def _wait_position(self, x, y, z, timeout=None):
        """等待到达位置"""
        if timeout is None:
            timeout = NavConfig.WAYPOINT_TIMEOUT

        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logwarn("[等待] 超时!")
                break

            cur = self.current_pose.pose.position
            dx = cur.x - x
            dy = cur.y - y
            dz = cur.z - z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist < NavConfig.ARRIVAL_THRESHOLD:
                break

            rospy.sleep(0.1)

    def _wait_altitude(self, altitude, timeout=15.0):
        """等待到达高度"""
        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logwarn("[等待] 高度超时!")
                break

            if abs(self.current_pose.pose.position.z - altitude) < 0.2:
                break

            rospy.sleep(0.1)

    # ============================================================================
    # 状态查询
    # ============================================================================
    def get_state(self):
        """获取当前状态"""
        pos = self.current_pose.pose.position
        tgt = self.target_pose.pose.position

        return {
            'nav_mode': self.mode.value,
            'mav_mode': self.mav_mode,
            'armed': self.is_armed,
            'position': (pos.x, pos.y, pos.z),
            'target': (tgt.x, tgt.y, tgt.z),
            'battery': self.battery_percentage,
            'has_vins': self.has_vins
        }

    def print_state(self):
        """打印状态"""
        state = self.get_state()

        print("\033[2J\033[H")  # 清屏
        print("=" * 40)
        print("  自主导航系统状态")
        print("=" * 40)
        print("  导航模式: {}".format(state['nav_mode']))
        print("  飞控模式: {}".format(state['mav_mode']))
        print("  电池: {:.0%}".format(state['battery']))
        print("-" * 40)
        print("  当前位置:")
        print("    X: {:.3f}m".format(state['position'][0]))
        print("    Y: {:.3f}m".format(state['position'][1]))
        print("    Z: {:.3f}m".format(state['position'][2]))
        print("-" * 40)
        print("  目标位置:")
        print("    X: {:.3f}m".format(state['target'][0]))
        print("    Y: {:.3f}m".format(state['target'][1]))
        print("    Z: {:.3f}m".format(state['target'][2]))
        print("=" * 40)


# ================================================================================
# 命令行接口
# ================================================================================
class NavCmd:
    """导航命令接口"""

    def __init__(self):
        self.nav = Navigator()

    def takeoff(self, altitude=None):
        self.nav.takeoff(altitude)

    def land(self):
        self.nav.land()

    def hold(self):
        self.nav.hold()

    def go_to(self, x, y, z=None):
        self.nav.go_to(x, y, z)

    def home(self):
        self.nav.go_home()

    def patrol(self, waypoints, loop=True):
        self.nav.set_waypoints(waypoints)
        self.nav.start_patrol(loop)

    def stop(self):
        self.nav.stop()

    def track(self):
        self.nav.enable_tracking()


# ================================================================================
# 主函数
# ================================================================================
def main():
    """主函数"""
    nav = Navigator()

    # 默认航点
    default_waypoints = [
        [3.0, 0.0, 1.2],
        [3.0, 3.0, 1.2],
        [0.0, 3.0, 1.2],
        [0.0, 0.0, 1.2]
    ]
    nav.set_waypoints(default_waypoints)

    rospy.loginfo("")
    rospy.loginfo("=" * 50)
    rospy.loginfo("  可用命令:")
    rospy.loginfo("=" * 50)
    rospy.loginfo("  nav.takeoff(altitude)    - 起飞")
    rospy.loginfo("  nav.land()              - 降落")
    rospy.loginfo("  nav.hold()             - 悬停")
    rospy.loginfo("  nav.go_to(x, y, z)     - 飞向位置")
    rospy.loginfo("  nav.go_home()          - 返航")
    rospy.loginfo("  nav.start_patrol()     - 开始巡逻")
    rospy.loginfo("  nav.stop()             - 停止(影子跟随)")
    rospy.loginfo("  nav.enable_tracking()  - Ego-Planner追踪")
    rospy.loginfo("=" * 50)
    rospy.loginfo("")

    rospy.spin()


if __name__ == '__main__':
    main()