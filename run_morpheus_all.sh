#!/bin/bash
# =============================================
#  Morpheus full-stack launcher
#  Author: SYH
# =============================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$SCRIPT_DIR/morpheus_ws"

source /opt/ros/humble/setup.bash
cd "$WORKSPACE"
source install/setup.bash

# morpheus_spawn.launch.py brings up the entire stack in one shot:
# Gazebo, controllers (via morpheus_control.launch.py), Nav2, sensor
# bridges, EKF, and — when ros2_aruco is installed — aruco_detector_2i
# with the correct parameters (marker_size, dictionary, camera topics).
echo "[Morpheus] Launching full stack (Gazebo + control + Nav2 + ArUco)..."
ros2 launch morpheus_simulation morpheus_spawn.launch.py "$@" &
SIM_PID=$!

trap 'kill "$SIM_PID" 2>/dev/null' EXIT
wait
