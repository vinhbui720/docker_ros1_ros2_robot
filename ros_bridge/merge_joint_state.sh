#!/usr/bin/env bash

ROS2_WS="/home/rosuser/ros2_ws"

# FIX: Properly check if EITHER package's install directory is missing
if [ ! -d "$ROS2_WS/install/gantry_controller" ] || [ ! -d "$ROS2_WS/install/env_joint_states_publish" ]; then

    echo "-------------------------------------------------------"
    echo "Controllers NOT found. Starting isolated build..."
    echo "-------------------------------------------------------"

    # Run the build process in a subshell so we don't pollute the current environment
    (
        # Source ROS 2 base (using setup.bash is generally safer here than local_setup)
        source /opt/ros/foxy/setup.bash

        cd "$ROS2_WS"
        echo "Building packages in $(pwd)..."
        
        # Build the controllers
        colcon build --symlink-install --packages-select gantry_controller env_joint_states_publish
    )
    
    echo "-------------------------------------------------------"
    echo "Build Complete. Environment is clean."
    echo "-------------------------------------------------------"
fi

# Source ROS 2 base and the local workspace
source /opt/ros/foxy/setup.bash
cd "$ROS2_WS"
source install/setup.bash 

# Launch the node
ros2 launch env_joint_states_publish joint_states_merger.launch.py