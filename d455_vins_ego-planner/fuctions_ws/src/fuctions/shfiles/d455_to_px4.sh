#!/bin/bash

# 终端1
xterm -hold -e bash -c "
    source /opt/ros/noetic/setup.bash;
    source ~/catkin_ws/devel/setup.bash;
    cd ~/catkin_ws;
    roslaunch realsense2_camera rs_camera.launch;
    exec bash
" &

sleep 15

# 终端2
xterm -hold -e bash -c "
    source /opt/ros/noetic/setup.bash;
    source ~/vins-fusion/devel/setup.bash;
    cd ~/vins-fusion;
    rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml;
    exec bash
" &

sleep 5

# 终端3
xterm -hold -e bash -c "
    source /opt/ros/noetic/setup.bash;
    roslaunch mavros px4.launch;
    exec bash
" &

sleep 5

# 终端4
xterm -hold -e bash -c "
    source /opt/ros/noetic/setup.bash;
    source ~/catkin_ws/devel/setup.bash;
    python3 ~/Desktop/vins.py;
    exec bash
" &

echo "终端已按顺序打开"
