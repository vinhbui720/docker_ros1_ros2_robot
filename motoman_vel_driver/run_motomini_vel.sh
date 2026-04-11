#!/usr/bin/env bash
# run_motomini_vel.sh
# Entrypoint for the motoman_vel_controller Docker container.
# Builds the catkin_vel_ws on first run, then launches the MotoMini
# velocity-streaming driver (real hardware or simulation).
#
# Environment variables (set in docker-compose or shell):
#   SIMULATION=true   -> start fake_motomini_node, skip real hardware
#   SIMULATION=false  -> connect to real YRC1000 (default)
#   ROBOT_IP          -> IP of YRC1000 controller (default: 192.168.1.14)

set -e

# ── 1. Source base ROS installation ────────────────────────────────────────────
source /opt/ros/noetic/setup.bash

# Ensure ROS logging goes to a writable location
export ROS_LOG_DIR="/home/rosuser/.ros/log"
mkdir -p "${ROS_LOG_DIR}"

# ── 2. Build workspace on first run ────────────────────────────────────────────
CATKIN_WS="/home/rosuser/catkin_vel_ws"

if [ ! -f "${CATKIN_WS}/devel/setup.bash" ]; then
    echo "-------------------------------------------------------"
    echo "First run detected: Building catkin_vel_ws ..."
    echo "-------------------------------------------------------"
    cd "${CATKIN_WS}"
    # Create the top-level CMakeLists.txt symlink if not present
    if [ ! -f src/CMakeLists.txt ]; then
        catkin_init_workspace src
    fi
    catkin_make
fi

# ── 3. Source local workspace ───────────────────────────────────────────────────
source "${CATKIN_WS}/devel/setup.bash"

# ── 4. Wait for ROS master ──────────────────────────────────────────────────────
echo "Waiting for ROS master at ${ROS_MASTER_URI:-http://localhost:11311} ..."
until rosparam get /rosdistro >/dev/null 2>&1; do
    sleep 1
done
echo "ROS master is available."

# ── 5. Resolve launch arguments from environment ────────────────────────────────
ROBOT_IP="${ROBOT_IP:-192.168.1.14}"
SIMULATION="${SIMULATION:-false}"

echo "-------------------------------------------------------"
echo "  SIMULATION : ${SIMULATION}"
if [ "${SIMULATION}" != "true" ]; then
    echo "  ROBOT_IP   : ${ROBOT_IP}"
fi
echo "-------------------------------------------------------"

# ── 6. Launch ───────────────────────────────────────────────────────────────────
exec roslaunch --wait --screen \
    motoman_motomini_support robot_interface_streaming_motomini_vel.launch \
    robot_ip:="${ROBOT_IP}" \
    simulation:="${SIMULATION}"
