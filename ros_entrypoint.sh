#!/bin/bash
set -e

# Source ROS 1 (Noetic) first
source /opt/ros/noetic/setup.bash

# Source ROS 2 (Foxy) second
# This allows the bridge to see packages from both environments
source /opt/ros/foxy/setup.bash

# Execute the command passed into the docker container
exec "$@"