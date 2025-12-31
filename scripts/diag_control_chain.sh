#!/bin/bash
# Diagnose the cmd_vel control chain to find where motion commands are lost.
# Run AFTER sending a navigation goal in RViz.
#
# Usage: ./scripts/diag_control_chain.sh

CONTAINER="morpheus-sim"

ROS_ENV="source /opt/ros/humble/setup.bash && \
         source /workspace/Morpheus/morpheus_ws/install/setup.bash"

# Helper: sample a topic for N seconds. timeout runs INSIDE the container so
# ROS env (sourced above) is in scope when ros2 is invoked.
sample_topic() {
    local topic="$1"
    local secs="$2"
    docker exec "$CONTAINER" bash -c "$ROS_ENV && timeout --preserve-status $secs ros2 topic echo $topic --no-arr 2>&1 | head -60"
}

echo "================================================================"
echo "Control Chain Diagnostic"
echo "================================================================"
echo ""

echo ">>> [1/5] Active topics (cmd_vel related):"
docker exec "$CONTAINER" bash -c "$ROS_ENV && ros2 topic list | grep -E 'cmd_vel|odom|controller|velocity'"
echo ""

echo ">>> [2/5] Publishers/Subscribers of /cmd_vel_nav (Nav2 output):"
docker exec "$CONTAINER" bash -c "$ROS_ENV && ros2 topic info /cmd_vel_nav -v" 2>&1 | head -30
echo ""

echo ">>> [3/5] Sampling /cmd_vel_nav for 3s (Nav2 output):"
out3=$(sample_topic /cmd_vel_nav 3)
if [ -z "$out3" ] || ! echo "$out3" | grep -q "linear:"; then
    echo "  (no Twist messages — Nav2 is NOT publishing velocity)"
else
    echo "$out3"
fi
echo ""

echo ">>> [4/5] Sampling /cmd_vel for 3s (twist_mux output → robot):"
out4=$(sample_topic /cmd_vel 3)
if [ -z "$out4" ] || ! echo "$out4" | grep -q "linear:"; then
    echo "  (no Twist messages — twist_mux is NOT forwarding)"
else
    echo "$out4"
fi
echo ""

echo ">>> [5/5] Sampling /forward_velocity_controller/commands for 3s (wheel commands):"
out5=$(sample_topic /forward_velocity_controller/commands 3)
if [ -z "$out5" ] || ! echo "$out5" | grep -q "data:"; then
    echo "  (no Float64MultiArray messages — robot_control is NOT publishing wheel commands)"
else
    echo "$out5"
fi
echo ""

echo "================================================================"
echo "Interpretation:"
echo "  - [3] empty → Nav2 controller failure (config / TF / costmap)"
echo "  - [3] OK, [4] empty → twist_mux broken or remap wrong"
echo "  - [4] OK, [5] empty → robot_control node down or deadzone too aggressive"
echo "  - [5] OK but rover not moving → ros2_control / Gazebo joints stuck"
echo "================================================================"
