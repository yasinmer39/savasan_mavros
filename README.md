# savasan_mavros
2024 Savaşan İHA yarışması için denenen, PX4 SITL ile konuşmak için MAVROS tabanlı ana kod döngüsü.

Use this shell script (.sh) in order to run gazebo simulation easily with fixed-winged aircraft(s):
#!/bin/bash

cd ~/PX4-Autopilot/
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo-classic
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 multi_uav_mavros_sitl.launch vehicle:=plane 
