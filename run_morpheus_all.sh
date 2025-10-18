#!/bin/bash
# =============================================
#  Oscar / Morpheus 全系统自动启动脚本
#  Author: SYH
# =============================================

# ---- paths ----
WORKSPACE=~/Documents/Master\ Thesis/morpheus_ws

# ---- env ----
source /opt/ros/humble/setup.bash
cd "$WORKSPACE"
source install/setup.bash

# ---- start gazebo (bg) ----
gnome-terminal -- bash -c "echo '[1/3] Launching Gazebo...'; \
ros2 launch morpheus_simulation morpheus_spawn.launch.py; \
exec bash"

# ---- start control (bg) ----
sleep 10
gnome-terminal -- bash -c "echo '[2/3] Launching morpheus_control...'; \
source /opt/ros/humble/setup.bash; \
cd '$WORKSPACE'; \
source install/setup.bash; \
ros2 launch morpheus_control morpheus_control.launch.py; \
exec bash"

# ---- start aruco (bg) ----
sleep 5
gnome-terminal -- bash -c "echo '[3/3] Launching ros2_aruco...'; \
source /opt/ros/humble/setup.bash; \
cd '$WORKSPACE'; \
source install/setup.bash; \
ros2 launch ros2_aruco aruco_recognition.launch.py; \
exec bash"

echo "✅ [Running] All nodes are up."

