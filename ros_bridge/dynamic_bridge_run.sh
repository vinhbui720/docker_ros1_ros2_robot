#!/usr/bin/env bash
set -e

# Source ROS 1 first
source /opt/ros/noetic/setup.bash

# Then source ROS 2
source /opt/ros/foxy/setup.bash

# Source your ROS 2 workspace (IMPORTANT if ros1_bridge was built there)
source /root/ros2_ws/install/setup.bash

# Run the dynamic bridge
ros2 run ros1_bridge dynamic_bridge
