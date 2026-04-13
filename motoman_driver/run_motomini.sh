#!/usr/bin/env bash
set -e

# 1. Source the main ROS installation
source /opt/ros/noetic/setup.bash

# Ensure roslaunch can always write logs as current user.
export ROS_LOG_DIR="/home/rosuser/.ros/log"
mkdir -p "${ROS_LOG_DIR}"

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

# 4. Wait for external ROS master started by docker-compose service 'roscore'.
echo "Waiting for ROS master at ${ROS_MASTER_URI:-http://localhost:11311} ..."
until rosparam get /rosdistro >/dev/null 2>&1; do
  sleep 1
done
echo "ROS master is available."

ROBOT_IP="${ROBOT_IP:-192.168.1.14}"
SIMULATION="${SIMULATION:-false}"
echo "-------------------------------------------------------"
echo "  SIMULATION : ${SIMULATION}"
if [ "${SIMULATION}" != "true" ]; then
    echo "  ROBOT_IP   : ${ROBOT_IP}"
fi
echo "-------------------------------------------------------"

# 5. Launch the Motoman interface
exec roslaunch --wait --screen motoman_motomini_support robot_interface_streaming_motomini.launch \
    robot_ip:=${ROBOT_IP} \
    controller:=yrc1000 \
    simulation:=${SIMULATION}