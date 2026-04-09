一、imu标定：
1、下载code_utils:
mkdir -p ~/imu_catkin_ws/src
cd ~/imu_catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source ~/imu_catkin_ws/devel/setup.bash

2、编译：
cd ~/imu_catkin_ws/src
git clone https://github.com/gaowenliang/code_utils.git
cd ..
catkin_make

(1)warn：backward.hpp报错。
解决：sudo apt-get install libdw-dev
(2)catkin_make时出现backward.hpp没有找到
解决：将sumpixel_test.cpp中# include "backward.hpp"改为：#include “code_utils/backward.hpp”
(3)‘CV_LOAD_IMAGE_GRAYSCALE’ was not declared in this scope
解决：cd /home/a123/imu_catkin_ws/src/code_utils/src											

sed -i 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g' sumpixel_test.cpp

sed -i 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' mat_io_test.cpp

sed -i 's/CV_MINMAX/cv::NORM_MINMAX/g' sumpixel_test.cpp

3、下载imu_utils:
cd ~/imu_catkin_ws/src/
git clone https://github.com/gaowenliang/imu_utils.git
cd ..
catkin_make

4、下载编译Kalibr

sudo apt update
sudo apt install libsuitesparse-dev
sudo apt install libv4l-dev
安装系统库

参考官方教程https://github.com/ethz-asl/kalibr/wiki/installation，建议用源码
验证安装：source ~/kalibr_workspace/devel/setup.bash
kalibr_calibrate_cameras --help
kalibr_calibrate_imu_camera --help

5、标定：
(1)找到realsense-ros包，进入/catkin_ws/src/realsense-ros/realsense2_camera/launch（路径仅供参考），修改其中的rs_camera.launch的参数：\
Ⅰ.打开双目，修改图片的width、height
<arg name="infra_width" default="848"/>
<arg name="infra_height" default="480"/>
<arg name="enable_infra" default="true"/>
<arg name="enable_infral" default="true"/>
<arg name="enable_infra2" default="true"/>
<arg name="infra_rgb" default="false"/>
Ⅱ.打开imu的加速度计和陀螺仪并合并为一个topic
<arg name="enable_gyro" default="true"/>
<arg name="enable_accel" default="true"/>
<arg name="unite_imu method" default="linear_interpolation"/>
(2)启动命令为roslaunch realsense2_camera rs_camera.launch
(3)编辑启动文件:
gedit ~/imu_catkin_ws/src/imu_utils/launch/d455_imu_calibration.launch
写入：
<launch>

    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
    	<!--TOPIC名称和上面一致-->
        <param name="imu_topic" type="string" value= "/camera/imu"/>
        <!--imu_name 无所谓-->
        <param name="imu_name" type="string" value= "d455"/>
        <!--标定结果存放路径-->
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <!--数据录制时间-min-->
        <param name="max_time_min" type="int" value= "50"/>
        <!--采样频率，即是IMU频率，采样频率可以使用rostopic hz /camera/imu查看，设置为200，为后面的rosbag play播放频率-->
        <param name="max_cluster" type="int" value= "200"/>
    </node>
    
</launch>
(4)录制imu数据包，d455静止放置，放置时间大于d455_imu_calibration.launch里的时间
rosbag record -O imu_calibration /camera/imu (imu_calibration是数据包的名字，话题为imu话题)
(5)运行校准程序:
source ~/imu_catkin_ws/devel/setup.bash
roslaunch imu_utils d455_imu_calibration.launch
cd 存放imu_calibration.bag的路径
rosbag play -r 200 imu_calibration.bag
结果d455_imu_param.yaml可在设定的目录内查看。

二、多(双)相机标定：
1、下载/购买标定板：https://github.com/ethz-asl/kalibr/wiki/downloads
warn：本项目使用棋盘格标定，aprilgrid与Checkerboard
(棋盘格)，Circlegrid等标定板yaml文件模板不一样 需要自行查找。
棋盘格类型请参考本项目yaml文件参考april_129_A4.yaml

2、关闭结构光：
roslaunch realsense2_camera rs_camera.launch
rosrun rqt_reconfigure rqt_reconfigure
打开后将camera->stereo_module中的emitter_enabled设置为off(0) 

3、标定：
(1)新开终端：rviz
在里面add rgb和双目对应的topic，/camera/color/image_raw、/camera/infra1/image_rect_raw、/camera/infra2/image_rect_raw
(2)修改相机帧数（官方推荐是4Hz，尽管实际频率不完全准确，但是不影响结果）
kalibr在处理标定数据的时候要求频率不能太高，一般为4Hz，我们可以使用如下命令来更改topic的频率，实际上是将原来的topic以新的频率转成新的topic，实际测试infra1才是对应左目相机
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /color
rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 4.0 /infra_left
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 4.0 /infra_right
(3)录制：rosbag record -O multicameras_calibration /infra_left /infra_right /color
后面三个topic就是转换频率后的topic
(4)固定标定板，移动d455,确保标定板在rviz三个图像中完整
(5)kalibr标定：
cd kalibr_workspace/
source /devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --target /home/april_129_A4.yaml \
    --bag /home/multicameras_calibration.bag \
    --models pinhole-radtan pinhole-radtan pinhole-radtan \
    --topics /infra_left /infra_right /color \
    --bag-from-to 3 202 \
    --show-extraction \
    --approx-sync 0.04
warn：–-target 是标定板的配置文件,棋格盘，注意targetCols和targetRows表示的是内侧角点的数量，不是格子数量。
–-bag  是录制的数据包
--models pinhole-radtan pinhole-radtan pinhole-radtan表示三个摄像头的相机模型和畸变模型
--topics /infra_left /infra_right /color表示三个摄像头对应的拍摄的数据话题
–-bag-from-to 3 202表示处理bag中3-202秒的数据
–-show-extraction表示显示检测特征点的过程
可以使用rosbag info 来参看录制的包的信息
(6)报错：
Ⅰ、cv_bridege not find
解决方法：
步骤 1：确认 cv_bridge 安装状态
检查是否已安装：rospack find cv_bridge
若未找到，需手动从源码编译：
wget https://codechina.csdn.net/mirrors/ros-perception/vision_opencv/-/archive/noetic/vision_opencv-noetic.zip
unzip vision_opencv-noetic.zip
cd vision_opencv-noetic/cv_bridge
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 ..
make
sudo make install

步骤 2：解决 Python 与 Boost 版本冲突
若报错 undefined symbol: _Py_fopenhz，说明 Python 与 Boost 版本不匹配：
# 查看 Boost 链接的 Python 版本
ls /usr/lib/x86_64-linux-gnu/libboost_python*
# 确认 Python 版本
python3 --version
版本对应后重新安装匹配的 Boost：
sudo apt-get install libboost-python1.71.0  # 对应 Python 3.8

步骤 3：配置 Kalibr 的 Python 解释器
gedit /home/orangepi/kalibr_workspace/src/kalibr/as_lan_offline_calibration/kalibr/python/kalibr_calibrate_cameras.py
在 Kalibr 启动脚本（如 kalibr_calibrate_cameras）首行指定 Python 路径：
#!/usr/bin/env python3
并设置环境变量，确保编译的 cv_bridge 优先级更高：
export PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH

步骤 4：修复导入顺序
在调用 cv_bridge 的 Python 文件中，调整导入顺序（两种说法，优先尝试先 cv2）：
gedit /home/orangepi/kalibr_workspace/src/kalibr/as_lan_offline_calibration/kalibr/python/kalibr_calibrate_cameras.py
import cv2        # 先加载
import cv_bridge

Ⅱ、cannot import name NavigationToolbar2Wx
解决办法：发现 matplotlib 中没有NavigationToolbar2Wx 而是换成了NavigationToolbar2WxAgg 所以修改源码，将PlotCollection.py中的NavigationToolbar2Wx换成NavigationToolbar2WxAgg
原来的PlotCollection.py
得到结果：multicameras_calibration-camchain.yaml (还有一个txt和pdf文件，文件夹不再添加)

三、IMU+双目相机标定：

1、编写chain.yaml，格式参考Kalibr官方教程https://github.com/ethz-asl/kalibr/wiki/yaml-formats 中的chain.yaml，具体的参数参考上面得到的多目标定的yaml文件，其中cam0和cam1分别指的是红外左右目，cam1和没有的参数可以删除。

2、编写imu.yaml，格式参考https://github.com/ethz-asl/kalibr/wiki/yaml-formats 中的imu.yaml，具体参数使用之前imu标定得到的参数、没有的参数可以删除。

3、准备好april_129_A4.yaml

5、启动realsense：
roslaunch realsense2_camera rs_camera.launch

6、关闭结构光(同上)

7、打开rviz，add imu topic和infra1 topic以及infra2 topic，同时调整realsense位置，要确保双目图像数据一直包含标定板全部内容

8、调整imu和双目topic的发布频率以及以新的topic名发布它们，其中双目图像的发布频率改为20Hz，imu发布频率改为200Hz：
rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 20.0 /infra_left
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 20.0 /infra_right
rosrun topic_tools throttle messages /camera/imu 200.0 /imu

9、录制：(90s以上)
rosbag record -O imu_stereo.bag /infra_left /infra_right /imu

10、标定：
 rosrun kalibr kalibr_calibrate_imu_camera --bag /home/imu_stereo.bag --cam /home/chain.yaml --imu /home/imu.yaml --target /home/april_129_A4.yaml --bag-from-to 2 146 --show-extraction
最终得到的结果为yaml，txt，和pdf文件
标定结果的好坏可以看results-imucam-homezjimu_stereo.txt中的重投影误差Reprojection error，两个相机都在0.15以下说明标定的结果比较好

四、修改参数：
1、根据联合标定结果中imu_stereo-camchain-imucam.yaml和imu_stereo-imu.yaml的结果
填写到realsense_stereo_imu_config.yaml中，我的结果如下其中
（1）cam0：T_cam_imu表示的是imu到相机的变换矩阵，我们填写到body_T_cam0的矩阵应该是相机到imu的矩阵，所以对T_cam_imu矩阵取逆，因为旋转矩阵是正交矩阵，它的逆矩阵就等于它的转置矩阵，平移向量的逆，三轴取反就可以了，cam1的同理
（2）加速度计和陀螺仪的噪声和随机游走填入相应位置
2、填写相机配置文件left.yaml和right.yaml
根据camchain-imucam-homezjimu_stereo.yaml中的相机内参和畸变参数填写，结果如下：
（1）left.yaml
（2）right.yaml

WARN：可以直接查看和使用官方标定好的left.yaml、right.yaml的内参数据:
在realsense-ros工作空间下：roslaunch realsense2_camera rs_camera.launch
Ⅰ、左目：rostopic echo /camera/infra1/camera_info
Ⅱ、右目：rostopic echo /camera/infra2/camera_info
得到：[fx,cx]
      [fy,cy]的参数矩阵
直接修改即可.

五、飞控与d455的外参矩阵标定：
1、启动px4:
roslaunch mavros Px4.launch
2、修改realsense_stereo_imu_config.yaml中的imu话题，改为：imu/mavros/data_raw
并根据realsense_stereo_imu_config.yaml里面outpath的路径建文件夹
3、先测量d455相对于飞控的实际物理偏差值填入~/yourpath/realsense_stereo_imu_config.yaml并将load_previous_pose_graph参数改为1
启动realsense_stereo_imu_config.yaml：
rosrun ~/yourpath/realsense_stereo_imu_config.yaml
4、查看飞机位姿信息：
rostopic echo /vins_fusion/imu_propagate
5、拿着飞机在空旷区域缓慢转两圈，最后把飞机放回起飞位置，查看飞机位姿的偏移量，终止程序，在output文件夹里面会生成vio文件，将里面的外参矩阵数据覆盖到realsense_stereo_imu_config.yaml里的参数矩阵。
6、重复操作直到偏移量达到自己的要求。

WARNNNN：飞控不好用d455的imu

参考：https://blog.csdn.net/qq_40186909/article/details/113104595
感谢这篇文章的指导，感激不尽。
