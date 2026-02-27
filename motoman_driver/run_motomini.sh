#!/usr/bin/env bash
set -e

# Source ROS Noetic
source /opt/ros/noetic/setup.bash

# Source your catkin workspace
source /root/catkin_ws/devel/setup.bash

# Launch Motoman Motomini
roslaunch motoman_motomini_support robot_interface_streaming_motomini.launch \
  robot_ip:=192.168.1.11 \
  controller:=yrc1000
