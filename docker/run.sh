#!/bin/bash
# Build (if needed) and enter the Morpheus simulation container.
#
# - Mounts the project directory into the container so the code,
#   `colcon build` artifacts and assets all live on the host.
# - Passes through the NVIDIA GPU (for Gazebo/RViz rendering) and the X11
#   display (for GUI windows) and the joystick device (for teleoperation).
set -e

IMAGE_NAME=morpheus-sim
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t "$IMAGE_NAME" \
    "$PROJECT_DIR/docker"

# Allow local containers to connect to the host's X server (GUI passthrough).
xhost +local:docker > /dev/null

docker run -it --rm \
    --gpus all \
    --network host \
    --ipc host \
    --env DISPLAY="$DISPLAY" \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$PROJECT_DIR":/workspace/Morpheus \
    --device /dev/input \
    --name morpheus-sim \
    "$IMAGE_NAME" \
    bash
