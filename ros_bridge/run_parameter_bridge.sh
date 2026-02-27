#!/usr/bin/env bash
set -e

############################
# 1. ROS 1 environment
############################
source /opt/ros/noetic/setup.bash
source /home/rosuser/catkin_ws/devel/setup.bash

# Load bridge params into ROS 1 master
rosparam load /home/rosuser/catkin_ws/bridge.yaml

############################
# 2. Clean ROS 1 vars BEFORE ROS 2
############################
unset ROS_PACKAGE_PATH
unset ROS_ROOT
unset ROS_VERSION
unset ROS_PYTHON_VERSION

############################
# 3. ROS 2 environment
############################
source /opt/ros/foxy/setup.bash
source /home/rosuser/ros2_ws/install/setup.bash

############################
# 4. Run bridge
############################
ros2 run ros1_bridge parameter_bridge