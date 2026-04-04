一、安装mavros：
# 1. 安装 mavros 及扩展包
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs

# 2. 安装 GeographicLib 数据集（必须，否则节点无法启动）
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

# 确认包已安装
rospack find mavros
# 查看可用 launch 文件（APM / PX4）
ls /opt/ros/noetic/share/mavros/launch/

二、驱动sdk安装：
ubuntu20.04(noetic)
D455固件版本：5.15.0.2
固件官网下载：https://dev.realsenseai.com/docs/firmware-releases-d400

1、依赖安装：
sudo apt update
sudo apt install git cmake g++ libssl-dev libusb-1.0-0-dev
libudev-dev pkg-config libgtk-3-dev libglfw3-dev -y

2、驱动下载：
git clone https://github.com/realsenseai/librealsense.git（用的是V2.57.7版本）

3、编译：
cd librealsense
sudo ./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=true
cmake .. \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DCMAKE_BUILD_TYPE=release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_TOOLS=true
make -j$(nproc)
sudo make install
sudo ldconfig

4、检验：
rs-enumerate-devices -o  查看是否有输出

5、常见问题：
（1）虽然lsusb识别到了，但realsense-viewer这个工具识别不到 （这个一般是SDK版本与固件版本不配备，这个自己试,还有试试插拔）
（2）realsense-viewer这个工具识别到了，但IMU没了，这个是用在cmake后面加-DFORCE_RSUSB_BACKEND=true（用USB后端编译）

6、rs_camera.launch修改参数
1  <arg name="enable_infra"        default="true"/>

2  <arg name="enable_infra1"       default="true"/>

3  <arg name="enable_infra2"       default="true"/>

4  <arg name="enable_gyro"         default="true"/>

5  <arg name="enable_accel"        default="true"/>

6  <arg name="enable_sync"         default="true"/>

7  <arg name="align_depth"         default="true"/>

三、realsense的ros包安装：

# 1. 克隆 realsense-ros 到工作空间
cd ~/realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros1-legacy

# 2. 安装依赖并编译
cd ~/realsense_ws
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash

四、安装vins-fusion：
1、安装依赖：
# Ceres Solver（必须）
sudo apt-get install -y libceres-dev
# OpenCV（Noetic 默认已带 4.x，一般无需额外安装）
sudo apt-get install -y libopencv-dev
# 其他依赖
sudo apt-get install -y libeigen3-dev libboost-all-dev

2、克隆源码：
cd ~/vins_ws/src
# 官方仓库
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
3、编译：
cd ~/catkin_ws
catkin_make -j4
source devel/setup.bash
常见问题：Ⅰ、error: 'CV_LOAD_IMAGE_GRAYSCALE' was not declared
解决方法：将代码中旧 API 替换：
// 旧（OpenCV 3）
CV_LOAD_IMAGE_GRAYSCALE  →  cv::IMREAD_GRAYSCALE
CV_BGR2GRAY              →  cv::COLOR_BGR2GRAY
CV_GRAY2RGB              →  cv::COLOR_GRAY2BGR
或者直接用打好补丁的分支：
git clone https://github.com/zinuok/VINS-Fusion.git  # 社区修复版
Ⅱ、cv_bridge 与 OpenCV 4 冲突
解决：sudo apt-get install ros-noetic-cv-bridge
Ⅲ、loop_fusion 模块编译失败
解决：catkin_make --pkg camera_models vins loop_fusion -j4

4、验证：
# 终端1：启动 roscore
roscore
# 终端2：运行 VINS-Fusion
rosrun vins vins_node \
  ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
# 终端3：播放数据包
rosbag play YOUR_EUROC_BAG.bag

5、目录结构说明：
VINS-Fusion/
├── camera_models/     # 相机模型
├── vins_estimator/    # 核心估计器
├── loop_fusion/       # 回环检测
├── global_fusion/     # GPS 融合（可选）
└── config/            # 各传感器配置文件
    ├── euroc/
    ├── realsense_d435i/   ← 你用 RealSense 的话用这个
    └── ...
    
四.五、如果还提示ceres-solver没有安装先安装ceres：
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local   # 可指定安装路径，例如 /usr/local
make -j$(nproc)   # 使用所有CPU核心并行编译
make test         # 可选，运行测试套件
sudo make install # 安装到系统目录

五、根据kalib_标定进行标定

六、测试：
第一：
1、打开一个终端 ，用roslaunch 打开realsense摄像头：
source ~/catkin_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
2、新开终端：
source ~/catkin_ws/devel/setup.bash
roslaunch vins vins_rviz.launch
3、新开终端：
source ~/catkin_ws/devel/setup.bash
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/yourconfig_path/your_config_file.yaml 
绕着走了两个矩形，理论上打开回环检测模块，结果会更加精确
注意，要关闭结构光，结构光会影响轨迹的准确性
结果可以参考主目录里的：data-test-iamges.jpg

第二：
启动：
roslaunch realsense2_camera rs_camera.launch
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/yourconfig_path/your_config_file.yaml 
roslaunch mavros px4.launch
python3 vins-to-px4.py
观看/mavros/local_position/pose:
rostopic echo /mavros/local_position/pose
观察飞机转一圈过程中位姿是否出现漂移，回到原点后位姿是否归零.
eg:
/mavros/vision_pose/pose输入飞控EKF2融合后告诉飞控:“我认为你现在在(2,y，z)。
/mavros/local_position/pose飞控最终确认的:"结合了视觉、IMU、气压计，我确定我在(a,y,z)。

七、ego-planner安装：
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

测试：
roslaunch ego_planner single_run_in_sim.launch
