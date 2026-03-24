#!/usr/bin/env bash
set -e

# 1. Source the main ROS installation
source /opt/ros/noetic/setup.bash

# 2. Check if the workspace has been built yet
if [ ! -f "/home/rosuser/catkin_ws/devel/setup.bash" ]; then
    echo "-------------------------------------------------------"
    echo "First run detected: Configuring and Building..."
    echo "-------------------------------------------------------"
    cd /home/rosuser/catkin_ws
    
    # Force the workspace to recognize the new home path
    catkin config --init --mkdirs 
    catkin build 
fi

# 3. Source your local workspace
source /home/rosuser/catkin_ws/devel/setup.bash

# 4. Launch the Motoman interface
roslaunch motoman_motomini_support robot_interface_streaming_motomini.launch \
  robot_ip:=192.168.1.14 \
  controller:=yrc1000