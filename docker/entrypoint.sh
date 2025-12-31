#!/bin/bash
set -e

# Clean stale FastRTPS shared-memory files left by previous runs
# (--ipc host shares /dev/shm with the host; orphaned lock files
# prevent new DDS participants from initialising).
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

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
