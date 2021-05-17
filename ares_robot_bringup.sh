#!/bin/bash

#chmod 666 /dev/ttyACM0

#roslaunch ares_bringup ares.launch

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#export ROS_MASTER_URL=http://192.168.1.121:11311
#export ROS_HOSTNAME=192.168.1.117

roslaunch ares_bringup ares_remote.launch
