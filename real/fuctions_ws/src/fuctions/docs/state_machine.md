# 自主导航避障系统

基于 **VINS-Fusion + Ego-Planner** 的无人机自主巡航系统。

## 硬件配置

| 组件 | 型号 |
|------|------|
| 机载电脑 | OrangePi 5 Max |
| 相机 | Intel RealSense D455 |
| 飞控 | PX4 (微控743) |
| 系统 | Ubuntu 20.04 + ROS Noetic |

## 系统架构

```
┌─────────────────────────────────────────────────┐
│                 自主导航系统                      │
├─────────────────────────────────────────────────┤
│  命令层: ROS Service (/nav/takeoff等)           │
│  状态机: SHADOW→TAKEOFF→HOLD→TRACK/WAYPOINT     │
│  规划层: Ego-Planner 实时避障                   │
│  控制层: MAVROS → PX4 飞控                    │
├─────────────────────────────────────────────────┤
│  定位层: VINS-Fusion (D455 + IMU)               │
└─────────────────────────────────────────────────┘
```

## 功能特性

- **多模式状态机**: 待机/起飞/悬停/航点/巡逻/返航
- **Ego-Planner避障**: 实时路径规划与避障
- **影子跟随**: 遥控器切换无顿挫
- **安全保护**: VINS检查/高度限制/电池保护

## 启动 (6个终端)

```bash
# 终端1: 相机
roslaunch realsense2_camera rs_camera.launch

# 终端2: 飞控通信
roslaunch mavros px4.launch

# 终端3: VINS定位
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/xxx/xxx.yaml

# 终端4: VINS→PX4桥接
python3 ~/d455_vins_ego-planner/fuctions_ws/src/fuctions/scripts/vins-to-px4.py

# 终端5: Ego-Planner避障
roslaunch fuctions ego/run_in_sim.launch

# 终端6: 自主导航
python3 ~/d455_vins_ego-planner/fuctions_ws/src/fuctions/scripts/autonomous_navigator.py
```

## ROS命令 (全部)

| 命令 | 功能 | 示例 |
|------|------|------|
| `/nav/takeoff` | 起飞（默认高度） | `rosservice call /nav/takeoff "{}"` |
| `/nav/go_to` | 飞向下一个预置航点 | `rosservice call /nav/go_to "{}"` |
| `/nav/return_home` | 返航（默认高度） | `rosservice call /nav/return_home "{}"` |
| `/nav/land` | 降落 | `rosservice call /nav/land "{}"` |
| `/nav/hold` | 悬停 | `rosservice call /nav/hold "{}"` |
| `/nav/start_patrol` | 开始巡逻 | `rosservice call /nav/start_patrol "{}"` |
| `/nav/enable_tracking` | 启用避障追踪 | `rosservice call /nav/enable_tracking "{}"` |
| `/nav/stop` | 停止(切影子) | `rosservice call /nav/stop "{}"` |
| `/nav/status` | 查看状态 | `rosservice call /nav/status "{}"` |

## Python接口

```python
nav = Navigator()

# 基础命令
nav.takeoff(1.5)           # 起飞到1.5米
nav.go_to(3, 0, 1.2)       # 飞向目标(启用避障)
nav.go_home()              # 返航回家
nav.land()                 # 降落
nav.hold()                 # 悬停当前位置

# 高级命令
nav.enable_tracking()      # 启用Ego-Planner避障追踪
nav.disable_tracking()    # 禁用避障
nav.start_patrol()        # 开始巡逻(自动遍历航点)
nav.stop_patrol()         # 停止巡逻
nav.stop()               # 停止控制，切回影子跟随

# 状态
nav.set_waypoints([[2,0,1.2], [2,2,1.2], [0,2,1.2], [0,0,1.2]])
nav.print_state()         # 打印状态
```

## 飞行流程

```
1. 遥控器手动起飞 → 悬停
2. 切换到 OFFBOARD 模式
3. 使用ROS命令控制飞行
4. 飞行中可随时切回遥控器(影子模式)
```

## enable_tracking (避障追踪) 详解

**是的,`enable_tracking` 就是启用 Ego-Planner 避障追踪:**

- 启用后,飞机会实时获取 Ego-Planner 的避障规划路径
- 飞行中检测到障碍物会自动绕行
- 配合 `go_to` 命令使用效果最佳

```python
# 推荐飞行方式:
nav.takeoff(1.5)              # 起飞
nav.enable_tracking()          # 启用避障
nav.go_to(5, 3, 1.5)        # 飞向目标(自动避障)
```

## 模式说明

| 模式 | 说明 |
|------|------|
| SHADOW | 影子跟随,遥控器控制 |
| TAKEOFF | 起飞中 |
| HOLD | 悬停 |
| WAYPOINT | 航点飞行 |
| PATROL | 巡逻模式 |
| TRACK | Ego-Planner避障追踪 |
| RETURN | 返航中 |
| LAND | 降落中 |

## 配置参数

在 `NavConfig` 类中修改:

```python
class NavConfig:
    MIN_ALTITUDE = 0.5       # 最低高度 (m)
    DEFAULT_ALTITUDE = 1.2   # 默认悬停高度 (m)
    TAKEOFF_ALTITUDE = 1.5  # 起飞高度 (m)
    MAX_ALTITUDE = 10.0     # 最高高度 (m)
    MAX_VELOCITY = 0.5      # 最大速度 (m/s)
    SMOOTH_FACTOR = 0.3    # 平滑因子
    LOW_BATTERY_THRESHOLD = 0.20   # 低电量警告 (20%)
    CRITICAL_BATTERY = 0.15        # 强制返航 (15%)
    ARRIVAL_THRESHOLD = 0.3        # 到达判定 (m)
    PATROL_DWELL_TIME = 2.0        # 巡逻停留时间 (s)
```

## 安全注意事项

1. **VINS必须先启动**: 否则所有命令会被拒绝
2. **OFFBOARD模式**: 切到OFFBOARD后才能用命令控制
3. **最低高度**: 限制在0.5m防止钻地
4. **电池保护**: 15%自动强制返航
5. **遥控器随时可切**: 切出OFFBOARD即进入影子模式

## 快速开始

```bash
# 1. 启动所有6个终端(见上文)

# 2. 遥控器起飞→悬停→切OFFBOARD

# 3. 测试命令
rosservice call /nav/status "{}"            # 查看状态
rosservice call /nav/takeoff "{}"           # 起飞
rosservice call /nav/go_to "{}"             # 飞向下一个预置航点(避障)
rosservice call /nav/return_home "{}"       # 返航
rosservice call /nav/land "{}"              # 降落

# 或者巡逻
rosservice call /nav/start_patrol "{}"         # 开始巡逻
```

## 故障排除

| 问题 | 原因 | 解決 |
|------|------|------|
| 命令无响应 | 未切OFFBOARD | 遥控器切到OFFBOARD |
| VINS未启动 | VINS未运行 | 检查终端3 |
| 起飞被拒绝 | VINS未就绪 | 等待VINS初始化 |
| 飞机不受控 | 未发心跳包 | 检查ego_bridge.py |
| 避障不生效 | 未enable_tracking | 先调用enable_tracking |
