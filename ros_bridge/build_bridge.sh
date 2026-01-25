#!/bin/bash
set -e  # Stop the script immediately if any command fails

# 1. Define the correct directory
# I removed '/root/' assuming your workspace is directly in your home folder.
# If your folder is actually named 'ros_bridge', keep it. 
# If it is 'ros1_bridge_ws', change it below.
WORKSPACE_DIR=~/ros2_ws

echo "Navigating to workspace: $WORKSPACE_DIR"

# Check if the folder actually exists before trying to enter it
if [ ! -d "$WORKSPACE_DIR" ]; then
  echo "Error: Directory $WORKSPACE_DIR does not exist!"
  echo "Please check if you created the folder or check the path."
  exit 1
fi

cd "$WORKSPACE_DIR"

# 2. Source the environments (Order matters!)
echo "Sourcing ROS 1 (Noetic)..."
source /opt/ros/noetic/setup.bash

echo "Sourcing ROS 2 (Foxy)..."
source /opt/ros/foxy/local_setup.bash

# 3. Source your custom workspaces
# (Using '|| true' ensures the script doesn't crash if these are fresh builds)
echo "Sourcing custom workspaces..."
source ~/catkin_ws/devel/setup.bash || echo "Warning: catkin_ws setup not found."
source ~/ros2_ws/install/local_setup.bash || echo "Warning: ros2_ws setup not found."

# 4. Build the bridge
# Added explicit output to see what is happening
echo "Building ros1_bridge..."
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

echo "Build complete!"