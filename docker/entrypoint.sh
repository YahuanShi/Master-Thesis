#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Patched slam_toolbox
if [ -f /tmp/slam_toolbox_src/install/setup.bash ]; then
  source /tmp/slam_toolbox_src/install/setup.bash
fi

# Source workspace if it has been built
if [ -f /workspace/Morpheus/morpheus_ws/install/setup.bash ]; then
  source /workspace/Morpheus/morpheus_ws/install/setup.bash
fi

exec "$@"
