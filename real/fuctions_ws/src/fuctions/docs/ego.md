ego-planner安装：
mkdir -p ~/ego_ws/src
cd ~/ego_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ~/ego_ws
catkin_make
source devel/setup.bash

测试：
roslaunch ego_planner single_run_in_sim.launch

避障测试：
方法有很多种，这里选用最保险安全的一种：
在飞机前方2m左右放一个障碍物
首先在ego_ws/src/planner/plan_manage下创建一个scripts目录
这个文件里scripts里的ego_bridge.py文件移动到ego_ws/src/planner/plan_manage/scrpits里
重新编译egoplanner
terminal：
roslaunch realsense2_camera rs_camera.launch
terminal2：
cd vins-fusion
source ./devel/setup.bash
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/yourconfig_path/your_config_file.yaml 
terminal3:
roslaunch mavros px4.launch
terminal4:
python3 ~/yourpath/d455_vins_ego-planner/fuctions_ws/src/fuctions/scripts/vins-to-px4.py
terminal5：
cd ~/ego_ws
source ./devel/setup.bash
python3 ego_ws/src/planner/plan_manage/scrpits/ego_bridge.py
遥控器起飞，稳定后、将遥控器切换至offboard模式：
terminal6：
cd ~/ego_ws
source ./devel/setup.bash
roslaunch ego_planner run_in_sim.launch 
ego的参数配置可以参考d455_vins_ego-planner/fuctions_ws/src/fuctions/launch/ego/run_in_sim.launch和advanced_param.xml
飞机会自主飞向正前方3m处，并且绕开障碍物。