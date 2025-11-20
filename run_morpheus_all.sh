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

# morpheus_spawn.launch.py already brings up Gazebo, the controllers
# (via morpheus_control.launch.py) and Nav2 — no need to launch them again.
echo "[1/2] Launching simulation (Gazebo + control + Nav2)..."
ros2 launch morpheus_simulation morpheus_spawn.launch.py &
SIM_PID=$!

sleep 10
echo "[2/2] Launching ArUco marker detection..."
ros2 launch ros2_aruco aruco_recognition.launch.py &
ARUCO_PID=$!

echo "✅ [Running] All nodes are up. Press Ctrl+C to stop."
trap 'kill "$SIM_PID" "$ARUCO_PID" 2>/dev/null' EXIT
wait
