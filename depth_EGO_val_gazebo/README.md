你需要已经安装好了 Ubuntu 20.04、ROS Noetic 以及 MAVROS。
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
# 推荐切换到稳定的 1.13 或 1.14 版本（以 1.13 为例）
git checkout v1.13.3
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
# 地理数据集
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
# 打开gazebo
# WARN：可以直接编译带深度相机的，但是测试几次有bug，问ai是写源码的程序员忘添加了，可以试试1.14版本
make px4_sitl gazebo_iris_depth_camera
# ORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
export PX4_SIM_MODEL=iris_depth_camera
make px4_sitl gazebo
# 链接ros
cd ~/PX4-Autopilot
source /opt/ros/noetic/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/sitl_gazebo/models
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris sdf:=$(pwd)/Tools/sitl_gazebo/models/iris_depth_camera/iris_depth_camera.sdf
# 测试
# 回到你刚才运行那一大串 roslaunch 命令的终端（就是那个一直在刷日志的黑窗口）。
# 把鼠标点进这个黑窗口，敲一下键盘的回车键（Enter）。
# 你会发现，光标最左边出现了一个 pxh> 的提示符。这就是 PX4 飞控的底层命令行！
# 在这里输入起飞命令并回车：
commander takeoff

# 安装ego
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# 从浙江大学 FAST 实验室的官方仓库克隆代码
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
# 测试
roslaunch ego_planner single_run_in_sim.launch

# 起飞规划避障准备：
# 打开gazebo
cd ~/PX4-Autopilot
source /opt/ros/noetic/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/sitl_gazebo/models
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris sdf:=$(pwd)/Tools/sitl_gazebo/models/iris_depth_camera/iris_depth_camera.sdf
# 在gazebo中断输入：
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set COM_OBL_RC_ACT 0
param set COM_RCL_EXCEPT 4
# 新开终端：
python3 bridge.py 
# 新开终端
# 开启ego：
cd ~/catkin_ws
roslaunch ego_planner gazebo_ego.launch
# 新开终端：
rosservice call /mavros/set_mode 0 "OFFBOARD"
rosservice call /mavros/cmd/arming True
# 飞机起飞悬停
# 利用2d nv 规划
# bridge.py 和 gazebo_ego.launch根据实际情况更改

# WARN:实际使用需要下载realsense的驱动和ros功能包，并做出调整，ubuntu一般用：librealsense-v2.55.1/realsense-ros-ros1-legacy此版本
