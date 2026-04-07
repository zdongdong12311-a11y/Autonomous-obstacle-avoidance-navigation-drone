#!/bin/bash
################################################################################
# 脚本名称: keep_terminal_open.sh
# 描述: 执行命令后保持终端常开
################################################################################

# 终端1：命令执行后保持打开
gnome-terminal -- bash -c "
	cd realsense_ws;
	source ./devel/setup.bash;
    roslaunch realsense2_camera rs_camera.launch;
    sleep 8;
    exec bash  # 保持终端打开
"

# 端2：命令执行后保持打开
gnome-terminal -- bash -c "
	cd vins_ws;
    source ./devel/setup.bash;
    rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml;
    sleep 5;
    exec bash  # 保持终端打开
"
gnome-terminal -- bash -c "
    roslaunch mavros px4.launch;
    sleep 5;
    exec bash  # 保持终端打开
  
"
gnome-terminal -- bash -c "
    python3 d455_vins_ego-planner/fuctions_ws/src/fuctions/scripts/vins-to-px4.py;
    exec bash  # 保持终端打开
  
"
echo "两个终端已打开，执行完命令后将保持打开状态"
