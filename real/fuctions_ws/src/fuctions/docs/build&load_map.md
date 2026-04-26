A. 安装RTAP-map ROS：
sudo apt update
sudo apt install ros-noetic-rtabmap-ros

B.
terminal1：
roslaunch realsense2_camera rs_camera.launch
terminal2：
roslaunch mavros px4.launch
terminal3:
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/xxx/xxx.yaml
terminal4:
python3 ~/d455_vins_ego-planner/fuctions_ws/src/fuctions/scripts/vins-to-px4.py
建图：
terminal5:
roslaunch rtabmap_launch build_map.launch
//////
建完后的图一般保存在~/.ros里面可以用
rtabmap-databaseViewer ~/.ros/rtabmap.db
查看
eg：每次建图后都会将原有的删除重新生成
加载地图：
roslaunch rtabmap_launch load_map.launch
可以用rviz查看，话题名称是cloud_map

常见问题：
VINS 输出的 frame_id 和 child_frame_id 全都是 "world"。
问题所在：在 RViz 的左上角，你的 Fixed Frame 设为了 world。如果你的机器人（无人机）坐标系也叫 world，那么 RViz 会认为“机器人在相对于自己移动”，结果就是坐标永远在 (0,0,0)。
现象：地图虽然在生成，但它是重叠在一起的，或者你移动相机时，RViz 里的坐标轴（TF）根本不跳动。

解决：
修改 VINS 源码：
1. 找到并打开文件
在终端进入该目录（或者用文件管理器进入 utility 文件夹）：
nano ~/catkin_ws/src/VINS-Fusion/vins_estimator/src/utility/visualization.cpp
2. 查找关键代码
在文件中搜索关键词 child_frame_id 或者 "world"。
你会找到类似下面这一段代码（通常在 pubOdometry 函数里）：
// 找到这一行（大约在第 150 行左右）
odom.child_frame_id = "world";
3. 修改代码
将这一行修改为你想要的名称（例如 vins_body）：
// 修改后
odom.child_frame_id = "vins_body";
同时，如果你在附近看到 odom.header.frame_id = "world";，这个不要动，保持 "world"。
4. 重新编译 VINS
修改保存后，必须回到工作空间根目录重新编译：
cd ~/catkin_ws
catkin_make
source devel/setup.bash

