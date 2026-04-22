#!/usr/bin/env python3
"""自主导航避障系统 - 仿真专用版本
直接使用 Gazebo Odometry 替代 VINS-Fusion
"""
import rospy, math, time, threading, copy
from collections import deque
from enum import Enum
from typing import Optional, List, Dict

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import State, BatteryStatus, SetMode, CommandBool
from mavros_msgs.srv import CommandTOL, SetMode as SetModeSrv, CommandBool as CommandBoolSrv
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Trigger, TriggerResponse


class NavConfig:
    MIN_ALTITUDE = 0.3
    DEFAULT_ALTITUDE = 1.2
    TAKEOFF_ALTITUDE = 1.5
    MAX_ALTITUDE = 10.0
    ARRIVAL_THRESHOLD = 0.3
    PATROL_DWELL_TIME = 2.0
    LOW_BATTERY_THRESHOLD = 0.20
    VINS_TIMEOUT = 3.0
    MAX_VELOCITY = 1.5


class NavMode(Enum):
    IDLE = "idle"
    SHADOW = "shadow"
    TAKEOFF = "takeoff"
    HOLD = "hold"
    WAYPOINT = "waypoint"
    PATROL = "patrol"
    TRACK = "track"
    RETURN = "return"
    LAND = "land"


class NavigatorSim:
    def __init__(self):
        rospy.init_node('autonomous_navigator_sim')
        self._lock = threading.Lock()

        self.mode = NavMode.SHADOW
        self.mav_state = State()
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"

        self.has_odom = True  # 仿真始终有 odom
        self.odom_last_update = time.time()
        self.battery_pct = 1.0
        self.planner_cmd = None
        self.use_ego_planner = False

        self.waypoints = []
        self.wp_index = 0
        self.last_wp_time = 0

        self._init_comms()

        # 一些默认航点
        self.waypoints = [
            [2.0, 0.0, 1.5],
            [2.0, 2.0, 1.5],
            [0.0, 2.0, 1.5],
            [0.0, 0.0, 1.5]
        ]

        rospy.loginfo("=" * 50)
        rospy.loginfo("  自主导航系统(仿真版) 已启动!")
        rospy.loginfo("=" * 50)

    def _init_comms(self):
        # 发布 target
        self.pub_setpoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # 订阅 Gazebo/Odometry
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self._velocity_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self._battery_cb)

        # Ego-planner 输出
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self._planner_cb)

        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self._control_loop)

        # 服务
        rospy.Service('/nav/takeoff', Trigger, self._srv_takeoff)
        rospy.Service('/nav/go_to', Trigger, self._srv_go_to)
        rospy.Service('/nav/hold', Trigger, self._srv_hold)
        rospy.Service('/nav/land', Trigger, self._srv_land)
        rospy.Service('/nav/return_home', Trigger, self._srv_return_home)
        rospy.Service('/nav/start_patrol', Trigger, self._srv_patrol)
        rospy.Service('/nav/stop', Trigger, self._srv_stop)
        rospy.Service('/nav/status', Trigger, self._srv_status)

    def _pose_cb(self, msg):
        with self._lock:
            self.current_pose = copy.deepcopy(msg)
            self.odom_last_update = time.time()

    def _velocity_cb(self, msg):
        pass  # 可用于速度限制

    def _state_cb(self, msg):
        with self._lock:
            self.mav_state = copy.deepcopy(msg)

    def _battery_cb(self, msg):
        with self._lock:
            self.battery_pct = msg.percentage

    def _planner_cb(self, msg):
        with self._lock:
            self.planner_cmd = copy.deepcopy(msg)

    def _control_loop(self, event):
        try:
            with self._lock:
                mode = self.mode
                mav_mode = self.mav_state.mode
                current_pose = copy.deepcopy(self.current_pose)
                target_pose = copy.deepcopy(self.target_pose)
                planner_cmd = copy.deepcopy(self.planner_cmd) if self.planner_cmd else None
                battery_pct = self.battery_pct
                waypoints = self.waypoints.copy()

            # 检查飞控模式
            if mav_mode != "OFFBOARD":
                self._sync_target_to_current()
                return

            # 模式处理
            if mode == NavMode.SHADOW:
                self._sync_target_to_current()

            elif mode == NavMode.TAKEOFF:
                if abs(current_pose.pose.position.z - target_pose.pose.position.z) < 0.2:
                    rospy.loginfo("[起飞] 已到达目标高度")
                    with self._lock:
                        self.mode = NavMode.HOLD

            elif mode == NavMode.WAYPOINT:
                if self._check_arrival():
                    rospy.loginfo("[航点] 到达目标点")
                    with self._lock:
                        self.mode = NavMode.HOLD

            elif mode == NavMode.RETURN:
                if self._check_arrival():
                    rospy.loginfo("[返航] 已返回出发点")
                    with self._lock:
                        self.mode = NavMode.HOLD

            elif mode == NavMode.PATROL:
                if self._check_arrival() and time.time() - self.last_wp_time > NavConfig.PATROL_DWELL_TIME:
                    with self._lock:
                        self.wp_index = (self.wp_index + 1) % len(waypoints)
                        self._set_target_from_wp(self.wp_index)
                    self.last_wp_time = time.time()

            elif mode == NavMode.TRACK:
                if planner_cmd and self.use_ego_planner:
                    self._apply_planner_cmd(planner_cmd)

            # 发布目标
            target_pose.header.stamp = rospy.Time.now()
            self.pub_setpoint.publish(target_pose)

            # 电池警告
            if battery_pct < NavConfig.LOW_BATTERY_THRESHOLD:
                rospy.logwarn_throttle(10, "[电池] 电量低: {:.0%}".format(battery_pct))

        except Exception as e:
            rospy.logerr("[控制] {}".format(e))

    def _apply_planner_cmd(self, cmd):
        safe_z = max(cmd.position.z, NavConfig.MIN_ALTITUDE)
        safe_z = min(safe_z, NavConfig.MAX_ALTITUDE)
        with self._lock:
            self.target_pose.pose.position.x = cmd.position.x
            self.target_pose.pose.position.y = cmd.position.y
            self.target_pose.pose.position.z = safe_z
            q = quaternion_from_euler(0, 0, cmd.yaw)
            self.target_pose.pose.orientation = Quaternion(*q)

    def _sync_target_to_current(self):
        with self._lock:
            self.target_pose.pose = copy.deepcopy(self.current_pose.pose)

    def _check_arrival(self) -> bool:
        c = self.current_pose.pose.position
        t = self.target_pose.pose.position
        dist = math.sqrt((c.x - t.x)**2 + (c.y - t.y)**2 + (c.z - t.z)**2)
        return dist < NavConfig.ARRIVAL_THRESHOLD

    def _set_target_from_wp(self, index):
        if index < len(self.waypoints):
            wp = self.waypoints[index]
            with self._lock:
                self.target_pose.pose.position.x = wp[0]
                self.target_pose.pose.position.y = wp[1]
                self.target_pose.pose.position.z = wp[2] if len(wp) > 2 else NavConfig.DEFAULT_ALTITUDE

    # ========== 服务处理 ==========
    def _srv_takeoff(self, req):
        if self.mav_state.mode != "OFFBOARD":
            return TriggerResponse(False, "请先切换 OFFBOARD 模式")
        self._do_takeoff(NavConfig.TAKEOFF_ALTITUDE)
        return TriggerResponse(True, "起飞")

    def _srv_go_to(self, req):
        try:
            parts = req.data.split()
            if len(parts) >= 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                self._do_go_to(x, y, z)
                return TriggerResponse(True, "飞向 ({}, {}, {})".format(x, y, z))
            return TriggerResponse(False, "格式: x y z")
        except:
            return TriggerResponse(False, "错误")

    def _srv_hold(self, req):
        self._do_hold()
        return TriggerResponse(True, "悬停")

    def _srv_land(self, req):
        self._do_land()
        return TriggerResponse(True, "降落")

    def _srv_return_home(self, req):
        alt = float(req.data) if req.data else NavConfig.DEFAULT_ALTITUDE
        self._do_return_home(alt)
        return TriggerResponse(True, "返航")

    def _srv_patrol(self, req):
        self.wp_index = 0
        self._set_target_from_wp(0)
        self.last_wp_time = time.time()
        with self._lock:
            self.mode = NavMode.PATROL
        return TriggerResponse(True, "巡逻")

    def _srv_stop(self, req):
        self.mode = NavMode.SHADOW
        self.use_ego_planner = False
        self._sync_target_to_current()
        return TriggerResponse(True, "停止")

    def _srv_status(self, req):
        pos = self.current_pose.pose.position
        return TriggerResponse(
            True,
            "模式:{} 位置:({:.1f},{:.1f},{:.1f})".format(
                self.mode.value, pos.x, pos.y, pos.z
            )
        )

    # ========== 命令实现 ==========
    def _do_takeoff(self, altitude: float):
        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = altitude
        self.target_pose.pose.orientation.w = 1.0
        with self._lock:
            self.mode = NavMode.TAKEOFF
        rospy.loginfo("[起飞] 目标高度: {:.1f}m".format(altitude))

    def _do_go_to(self, x: float, y: float, z: float):
        z = max(z, NavConfig.MIN_ALTITUDE)
        z = min(z, NavConfig.MAX_ALTITUDE)
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        with self._lock:
            self.mode = NavMode.WAYPOINT
        rospy.loginfo("[航点] 目标: ({}, {}, {})".format(x, y, z))

    def _do_hold(self):
        self._sync_target_to_current()
        with self._lock:
            self.mode = NavMode.HOLD
        rospy.loginfo("[悬停]")

    def _do_return_home(self, altitude: float):
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = altitude
        with self._lock:
            self.mode = NavMode.RETURN
        rospy.loginfo("[返航] 高度: {:.1f}m".format(altitude))

    def _do_land(self):
        with self._lock:
            self.mode = NavMode.LAND
        rospy.loginfo("[降落]")


if __name__ == '__main__':
    try:
        nav = NavigatorSim()
        rospy.loginfo("服务: /nav/takeoff, /nav/go_to, /nav/hold, /nav/land, /nav/return_home, /nav/start_patrol")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass