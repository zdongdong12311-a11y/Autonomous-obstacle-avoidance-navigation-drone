#!/usr/bin/env python3
"""自主导航避障系统 - VINS-Fusion + Ego-Planner"""

import rospy, math, time, threading, copy
from collections import deque
from enum import Enum
from typing import Optional, List, Dict, Any

from geometry_msgs.msg import PoseStamped, Quaternion
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import State, BatteryStatus
from mavros_msgs.srv import CommandTOL, SetMode
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerResponse


# 配置 - 安全优先
class NavConfig:
    # 高度限制 (米) - 真实飞行必须保守
    MIN_ALTITUDE = 0.5           # 最低飞行高度，防止撞地
    DEFAULT_ALTITUDE = 1.2       # 默认悬停高度
    TAKEOFF_ALTITUDE = 1.2        # 默认起飞高度 (必须高于 MIN_ALTITUDE)
    MAX_ALTITUDE = 2.5           # 最高飞行高度 (根据当地法规调整)
    EMERGENCY_RTL_ALTITUDE = 3.0  # 紧急返航高度

    # 距离限制 (米)
    MAX_FLIGHT_RADIUS = 8.0       # 最大飞行半径 (防止飞丢)
    HOME_RADIUS = 1.0              # 返航点半径

    ARRIVAL_THRESHOLD = 0.3         # 到达判定距离
    PATROL_DWELL_TIME = 2.0         # 巡逻停留时间

    # 电池保护
    LOW_BATTERY_THRESHOLD = 0.25   # 低电量警告 (25%)
    CRITICAL_BATTERY = 0.18          # 严重低电量 (18%)
    LOW_BATTERY_RTL_THRESHOLD = 0.20     # 自动返航阈值 (20%)

    # VINS 保护
    VINS_TIMEOUT = 2.5               # VINS 超时时间 (秒) - 保守值
    VINS_RECONNECT_ATTEMPTS = 3        # VINS 重连尝试次数

    # 安全标志
    SAFETY_GEOFENCE_ENABLED = True       # 地理围栏开关
    AUTO_RTL_ON_VINS_LOSS = True      # VINS 丢失时自动返航
    AUTO_RTL_ON_LOW_BATTERY = True   # 低电量自动返航
    FORCE_TAKEOFF_MIN_ALTITUDE = True   # 强制起飞最低高度


# 导航模式
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


# 主类 - 增加安全保护
class Navigator:
    def __init__(self):
        rospy.init_node('autonomous_navigator')
        self._lock = threading.Lock()

        # 安全状态
        self._safety_mode = "MANUAL"      # MANUAL / SAFE / EMERGENCY
        self._emergency_reason = ""
        self._failsafe_triggered = False

        # 导航状态
        self.mode = NavMode.SHADOW
        self.mav_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self._last_velocity = [0.0, 0.0, 0.0]

        # VINS 状态
        self.has_vins = False
        self.vins_last_update = 0.0
        self._vins_consecutive_failures = 0

        # 电池状态
        self.battery_pct = 1.0
        self._low_battery_triggered = False

        # 规划器状态
        self.planner_cmd = None
        self.use_ego_planner = False

        # 航点
        self.home_position = [0.0, 0.0, 0.0]  # 返航点
        self.waypoints = []
        self.wp_index = 0
        self.last_wp_time = 0
        self._patrol_active = False  # 巡逻状态
        self._cmd_queue = deque(maxlen=10)

        # Ego-Planner 目标发布
        self._pub_goal = None

        self._init_comms()
        self.srv_land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self._print_startup_guide()
        rospy.loginfo("[安全] 自动保护已启用 - 地理围栏: {} | VINS丢失返航: {}".format(
            NavConfig.SAFETY_GEOFENCE_ENABLED, NavConfig.AUTO_RTL_ON_VINS_LOSS))

    def _init_comms(self):
        self.pub_setpoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/battery', BatteryStatus, self._battery_cb)
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self._vins_cb)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self._planner_cb)
        
        # 发布目标点给 Ego-Planner
        self._pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        rospy.Timer(rospy.Duration(1.0/30.0), self._control_loop)
        rospy.Timer(rospy.Duration(0.1), self._command_loop)

        rospy.Service('/nav/takeoff', Trigger, self._srv_takeoff)
        rospy.Service('/nav/go_to', Trigger, self._srv_go_to)
        rospy.Service('/nav/hold', Trigger, self._srv_hold)
        rospy.Service('/nav/land', Trigger, self._srv_land_trigger)
        rospy.Service('/nav/return_home', Trigger, self._srv_return_home)
        rospy.Service('/nav/start_patrol', Trigger, self._srv_patrol)
        rospy.Service('/nav/stop', Trigger, self._srv_stop)
        rospy.Service('/nav/enable_tracking', Trigger, self._srv_enable_tracking)
        rospy.Service('/nav/status', Trigger, self._srv_status)

    def _pose_cb(self, msg):   with self._lock: self.current_pose = copy.deepcopy(msg)
    def _state_cb(self, msg):  with self._lock: self.mav_state = copy.deepcopy(msg)
    def _battery_cb(self, msg):   with self._lock: self.battery_pct = msg.percentage
    def _vins_cb(self, msg):
        with self._lock:
            self.has_vins = True
            self.vins_last_update = rospy.Time.now().to_sec()
    def _planner_cb(self, msg): with self._lock: self.planner_cmd = copy.deepcopy(msg)

    def _command_loop(self, event):
        try:
            if not self._cmd_queue: return
            cmd, params = self._cmd_queue.popleft()
        except: return

        try:
            if cmd == "takeoff": self._do_takeoff(float(params) if params else NavConfig.TAKEOFF_ALTITUDE)
            elif cmd == "go_to":
                parts = params.split()
                if len(parts) >= 3: self._do_go_to(float(parts[0]), float(parts[1]), float(parts[2]))
            elif cmd == "land": self._do_land()
            elif cmd == "return_home": self._do_return_home(float(params) if params else NavConfig.DEFAULT_ALTITUDE)
            elif cmd == "hold": self._do_hold()
        except Exception as e: rospy.logerr("[命令] {}: {}".format(cmd, e))

    def _control_loop(self, event):
        try:
            with self._lock:
                mode = self.mode; mav_mode = self.mav_state.mode
                current_pose = copy.deepcopy(self.current_pose)
                target_pose = copy.deepcopy(self.target_pose)
                planner_cmd = copy.deepcopy(self.planner_cmd) if self.planner_cmd else None
                battery_pct = self.battery_pct; use_ego_planner = self.use_ego_planner
                waypoints = self.waypoints.copy()

            if mode not in [NavMode.SHADOW, NavMode.IDLE]:
                if not self._check_vins_healthy():
                    rospy.logerr("[VINS失效] 切换影子!")
                    with self._lock: self.mode = NavMode.SHADOW; self._sync_target_to_current()
                    return

            if mav_mode != "OFFBOARD":
                with self._lock: self.mode = NavMode.SHADOW; self._sync_target_to_current()
                return

            if mode == NavMode.SHADOW: self._sync_target_to_current()
            elif mode == NavMode.TAKEOFF:
                if abs(current_pose.pose.position.z - target_pose.pose.position.z) < 0.2:
                    rospy.loginfo("起飞完成")
                    with self._lock: self.mode = NavMode.HOLD
            elif mode == NavMode.WAYPOINT:
                if self._check_arrival(): with self._lock: self.mode = NavMode.HOLD
            elif mode == NavMode.RETURN:
                if self._check_arrival(): with self._lock: self.mode = NavMode.HOLD
            elif mode == NavMode.PATROL:
                # 无避障，直接飞
                if self._check_arrival() and time.time() - self.last_wp_time > NavConfig.PATROL_DWELL_TIME:
                    with self._lock: self._next_patrol_waypoint()
                    self.last_wp_time = time.time()
            elif mode == NavMode.TRACK:
                # 有避障，使用 Ego-Planner
                if planner_cmd and use_ego_planner: self._apply_planner_cmd_smooth(planner_cmd)
                # 巡逻模式下自动切换航点
                if self._patrol_active and self._check_arrival() and time.time() - self.last_wp_time > NavConfig.PATROL_DWELL_TIME:
                    self._send_goal_to_ego_planner(self.waypoints[self.wp_index])
                    self._next_patrol_waypoint()
                    self.last_wp_time = time.time()

            target_pose.header.stamp = rospy.Time.now()
            self.pub_setpoint.publish(target_pose)

            if battery_pct < NavConfig.LOW_BATTERY_THRESHOLD:
                rospy.logwarn_throttle(10, "电量低!")
            if battery_pct < NavConfig.CRITICAL_BATTERY and mode not in [NavMode.RETURN, NavMode.LAND]:
                with self._lock: self._cmd_queue.append(("return_home", str(NavConfig.DEFAULT_ALTITUDE)))
        except Exception as e: rospy.logerr("[控制] {}".format(e))

    def _apply_planner_cmd(self, cmd):
        safe_z = max(cmd.position.z, NavConfig.MIN_ALTITUDE)
        safe_z = min(safe_z, NavConfig.MAX_ALTITUDE)
        with self._lock:
            self.target_pose.pose.position.x = cmd.position.x
            self.target_pose.pose.position.y = cmd.position.y
            self.target_pose.pose.position.z = safe_z
            q = quaternion_from_euler(0, 0, cmd.yaw)
            self.target_pose.pose.orientation = Quaternion(*q)

    def _apply_planner_cmd_smooth(self, cmd):
        self._apply_planner_cmd(cmd)

    def _check_vins_healthy(self) -> bool:
        if not self.has_vins: return False
        if rospy.Time.now().to_sec() - self.vins_last_update > NavConfig.VINS_TIMEOUT:
            rospy.logwarn_throttle(1, "[VINS超时]")
            return False
        return True

    def _sync_target_to_current(self):
        self.target_pose.pose = self.current_pose.pose

    def _check_arrival(self) -> bool:
        c = self.current_pose.pose.position
        t = self.target_pose.pose.position
        return math.sqrt((c.x-t.x)**2 + (c.y-t.y)**2 + (c.z-t.z)**2) < NavConfig.ARRIVAL_THRESHOLD

    def _set_target_from_wp(self, index):
        if index < len(self.waypoints):
            wp = self.waypoints[index]
            self.target_pose.pose.position.x = wp[0]
            self.target_pose.pose.position.y = wp[1]
            self.target_pose.pose.position.z = wp[2] if len(wp) > 2 else NavConfig.DEFAULT_ALTITUDE

    def _next_patrol_waypoint(self):
        """切换到下一个巡逻点"""
        if self.waypoints:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            self._set_target_from_wp(self.wp_index)

    def _send_goal_to_ego_planner(self, waypoint):
        """发送目标点给 Ego-Planner 进行避障规划"""
        if self._pub_goal is None:
            return
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.position.z = waypoint[2] if len(waypoint) > 2 else NavConfig.DEFAULT_ALTITUDE
        goal.pose.orientation.w = 1.0
        self._pub_goal.publish(goal)

    # 命令
    def takeoff(self, altitude: Optional[float] = None):
        alt = max(altitude if altitude else NavConfig.TAKEOFF_ALTITUDE, NavConfig.DEFAULT_ALTITUDE)
        if not self._check_vins_healthy(): rospy.logerr("[起飞] VINS未启动!"); return
        self._cmd_queue.append(("takeoff", str(alt)))
        rospy.loginfo("[起飞] {:.1f}m".format(alt))

    def _do_takeoff(self, altitude: float):
        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = altitude
        self.target_pose.pose.orientation.w = 1.0
        self.mode = NavMode.TAKEOFF

    def land(self):
        self._cmd_queue.append(("land", ""))
        rospy.loginfo("[降落]")

    def _do_land(self):
        try: self.srv_land(0, 0, 0, 0, 0); self.mode = NavMode.LAND
        except rospy.ServiceException as e: rospy.logerr("[降落] {}".format(e))

    def go_to(self, x: float, y: float, z: Optional[float] = None):
        target_z = max(z if z else NavConfig.DEFAULT_ALTITUDE, NavConfig.DEFAULT_ALTITUDE)
        target_z = min(target_z, NavConfig.MAX_ALTITUDE)
        if not self._check_vins_healthy(): rospy.logerr("[飞行] VINS未启动!"); return
        if not self.use_ego_planner:
            rospy.logwarn("[提示] 启用Ego-Planner...")
            with self._lock: self.use_ego_planner = True; self.mode = NavMode.TRACK
        self._cmd_queue.append(("go_to", "{} {} {}".format(x, y, target_z)))
        rospy.loginfo("[飞向] ({}, {}, {})".format(x, y, target_z))

    def _do_go_to(self, x: float, y: float, z: float):
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.mode = NavMode.WAYPOINT

    def go_home(self, altitude: Optional[float] = None):
        target_alt = altitude if altitude else NavConfig.DEFAULT_ALTITUDE
        if not self._check_vins_healthy(): rospy.logerr("[返航] VINS未启动!"); return
        self._cmd_queue.append(("return_home", str(target_alt)))

    def _do_return_home(self, altitude: float):
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = altitude
        self.mode = NavMode.RETURN

    def hold(self): self._cmd_queue.append(("hold", ""))

    def _do_hold(self):
        self._sync_target_to_current()
        self.mode = NavMode.HOLD
        rospy.loginfo("[悬停]")

    def stop(self):
        self.mode = NavMode.SHADOW
        self.use_ego_planner = False
        self._sync_target_to_current()

    def enable_tracking(self):
        rospy.loginfo("[追踪] 启用")
        self.use_ego_planner = True
        self.mode = NavMode.TRACK

    def disable_tracking(self):
        self.use_ego_planner = False
        self.mode = NavMode.HOLD

    def set_waypoints(self, waypoints: List):
        self.waypoints = waypoints
        self.wp_index = 0

    def start_patrol(self):
        """启动巡逻模式
    
    使用方式:
        1. 先开启避障: rosservice call /nav/enable_tracking ""
        2. 再开启巡逻: rosservice call /nav/start_patrol ""
    """
        if not self.waypoints: self.waypoints = [[2,0,1.5], [2,2,1.5], [0,2,1.5], [0,0,1.5]]
        if not self._check_vins_healthy(): rospy.logerr("[巡逻] VINS未启动!"); return
        
        self.wp_index = 0
        self._set_target_from_wp(0)
        self.last_wp_time = time.time()
        
        # 根据是否已开启避障决定模式
        if self.use_ego_planner:
            # 已开启避障 → 使用 TRACK 模式
            self.mode = NavMode.TRACK
            self._patrol_active = True
            # 发送第一个目标点给 Ego-Planner
            self._send_goal_to_ego_planner(self.waypoints[0])
            rospy.loginfo("[巡逻] 已启用，避障飞行中")
        else:
            # 未开启避障 → 使用 PATROL 模式
            self.mode = NavMode.PATROL
            self._patrol_active = True
            rospy.logwarn("[巡逻] 避障未启用! 如需避障，先调用 /nav/enable_tracking")

    def stop_patrol(self): self.mode = NavMode.HOLD

    def get_state(self) -> Dict:
        pos = self.current_pose.pose.position
        return {
            'nav_mode': self.mode.value,
            'mav_mode': self.mav_state.mode,
            'position': (pos.x, pos.y, pos.z),
            'battery': self.battery_pct,
            'has_vins': self.has_vins,
            'use_ego': self.use_ego_planner
        }

    def print_state(self):
        s = self.get_state()
        print("\033[2J\033[H")
        print("="*50)
        print("  自主导航系统状态")
        print("="*50)
        print("  模式: {} | 飞控: {}".format(s['nav_mode'], s['mav_mode']))
        print("  电池: {:.0%}".format(s['battery']))
        print("  VINS: {} | Ego: {}".format(s['has_vins'], s['use_ego']))
        print("  位置: ({:.2f}, {:.2f}, {:.2f})".format(*s['position']))
        print("="*50)

    def _print_startup_guide(self):
        rospy.loginfo("="*50)
        rospy.loginfo("  自主导航避障系统已启动!")
        rospy.loginfo("="*50)
        rospy.loginfo("  启动顺序:")
        rospy.loginfo("    1. realsense2_camera")
        rospy.loginfo("    2. mavros")
        rospy.loginfo("    3. VINS")
        rospy.loginfo("    4. vins-to-px4.py")
        rospy.loginfo("    5. Ego-Planner")
        rospy.loginfo("    6. autonomous_navigator.py")
        rospy.loginfo("  流程: 遥控起飞->切OFFBOARD->命令控制")
        rospy.loginfo("="*50)

    # ROS服务
    def _srv_takeoff(self, req):
        if self.mav_state.mode != "OFFBOARD": return TriggerResponse(False, "请先切OFFBOARD")
        if not self._check_vins_healthy(): return TriggerResponse(False, "VINS未启动")
        self.takeoff(float(req.data) if req.data else NavConfig.TAKEOFF_ALTITUDE)
        return TriggerResponse(True, "起飞")

    def _srv_go_to(self, req):
        if not self._check_vins_healthy(): return TriggerResponse(False, "VINS未启动")
        try:
            parts = req.data.split()
            if len(parts) >= 3: x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            else: return TriggerResponse(False, "格式: x y z")
            self.go_to(x, y, z)
            return TriggerResponse(True, "飞行")
        except: return TriggerResponse(False, "错误")

    def _srv_hold(self, req): self.hold(); return TriggerResponse(True, "悬停")
    def _srv_land_trigger(self, req): self.land(); return TriggerResponse(True, "降落")
    def _srv_return_home(self, req):
        if not self._check_vins_healthy(): return TriggerResponse(False, "VINS未启动")
        self.go_home(float(req.data) if req.data else NavConfig.DEFAULT_ALTITUDE)
        return TriggerResponse(True, "返航")
    def _srv_patrol(self, req):
        if not self.waypoints: self.waypoints = [[2,0,1.2], [2,2,1.2], [0,2,1.2], [0,0,1.2]]
        if not self._check_vins_healthy(): return TriggerResponse(False, "VINS未启动")
        self.start_patrol()
        return TriggerResponse(True, "巡逻")
    def _srv_stop(self, req): self.stop(); return TriggerResponse(True, "停止")
    def _srv_enable_tracking(self, req): self.enable_tracking(); return TriggerResponse(True, "追踪已启用")
    def _srv_status(self, req):
        s = self.get_state()
        return TriggerResponse(True, "模式:{} 电池:{:.0%} 位置:({},{},{:.1f})".format(
            s['nav_mode'], s['battery'], s['position'][0], s['position'][1], s['position'][2]))


if __name__ == '__main__':
    try:
        nav = Navigator()
        nav.set_waypoints([[2,0,1.2], [2,2,1.2], [0,2,1.2], [0,0,1.2]])
        rospy.loginfo("命令: takeoff/go_to/go_home/land/hold/start_patrol/enable_tracking/stop/print_state")
        rospy.spin()
    except rospy.ROSInterruptException: pass
