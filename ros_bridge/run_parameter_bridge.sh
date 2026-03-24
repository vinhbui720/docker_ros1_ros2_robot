#!/usr/bin/env bash
set -e

# --- 1. SETTINGS ---
CATKIN_WS="/home/rosuser/catkin_ws"
ROS2_WS="/home/rosuser/ros2_ws"

# --- 2. CHECK IF BRIDGE IS BUILT ---
if [ ! -d "$ROS2_WS/install/ros1_bridge" ]; then
    echo "-------------------------------------------------------"
    echo "Bridge NOT found. Starting isolated build..."
    echo "-------------------------------------------------------"

    # Use a subshell (parentheses) to keep this environment isolated
    (
        # Source ROS 1 first
        source /opt/ros/noetic/setup.bash
        if [ -f "$CATKIN_WS/devel/setup.bash" ]; then
            source "$CATKIN_WS/devel/setup.bash"
        fi

        # Source ROS 2 base (Local setup to avoid over-writing ROS 1 vars)
        source /opt/ros/foxy/local_setup.bash

        cd "$ROS2_WS"
        echo "Building bridge in $(pwd)..."
        
        # Build bridge. We use --cmake-force-configure to ensure it 
        # re-detects the ROS 1 packages we just sourced.
        colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
    )
    
    echo "-------------------------------------------------------"
    echo "Build Complete. Environment is clean."
    echo "-------------------------------------------------------"
fi

# --- 3. RUNNING THE BRIDGE ---
# We need to load ROS 1 params first
echo "Loading bridge parameters into ROS 1 Master..."
source /opt/ros/noetic/setup.bash
if [ -f "$CATKIN_WS/devel/setup.bash" ]; then
    source "$CATKIN_WS/devel/setup.bash"
fi
rosparam load "$CATKIN_WS/bridge.yaml"

# Now, we CLEAN the environment for the ROS 2 runtime
echo "Cleaning ROS 1 environment variables..."
unset ROS_PACKAGE_PATH
unset ROS_ROOT
unset ROS_VERSION
unset ROS_PYTHON_VERSION

# Source ROS 2 and the newly built bridge
source /opt/ros/foxy/setup.bash
source "$ROS2_WS/install/setup.bash"

echo "Starting ROS1-ROS2 Bridge..."
# exec replaces the shell process with the bridge
exec ros2 run ros1_bridge parameter_bridge
# exec ros2 launch env_joint_states_publish joint_states_merger.launch.py