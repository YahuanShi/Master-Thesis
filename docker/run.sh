#!/bin/bash
# =============================================
#  Morpheus — one-command environment setup
#
#  First run on a new machine:
#    ./docker/run.sh          # build image + start container + enter shell
#
#  Subsequent runs:
#    ./docker/run.sh          # start (if stopped) + enter shell
#    ./docker/run.sh build    # colcon build inside container
#    ./docker/run.sh test     # run tests inside container
#    ./docker/run.sh launch   # launch full stack
#    ./docker/run.sh clean    # wipe build cache and rebuild image
# =============================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTAINER="morpheus-sim"

cd "$SCRIPT_DIR"

# ROS + workspace sourcing prefix for all container commands
ROS_ENV="source /opt/ros/humble/setup.bash && \
  [ -f /tmp/slam_toolbox_src/install/setup.bash ] && source /tmp/slam_toolbox_src/install/setup.bash; \
  [ -f /workspace/Morpheus/morpheus_ws/install/setup.bash ] && source /workspace/Morpheus/morpheus_ws/install/setup.bash;"

# ── Helpers ──────────────────────────────────────────────────────────
log()  { echo -e "\033[1;36m[Morpheus]\033[0m $*"; }
die()  { echo -e "\033[1;31m[Morpheus]\033[0m $*" >&2; exit 1; }

ensure_container() {
  if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    return 0  # already running
  fi
  xhost +local:docker >/dev/null 2>&1 || true
  if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    # container exists but stopped — restart without rebuilding image
    log "Restarting container..."
    docker start "$CONTAINER"
  else
    # first time — build image and create container
    log "Creating container (first run, building image)..."
    HOST_UID="$(id -u)" HOST_GID="$(id -g)" docker compose up -d --build
  fi
  log "Container ready."
}

# Kill any running ROS stack and free all DDS discovery ports.
# Safe to call even when nothing is running.
kill_ros() {
  # Kill the launch process tree inside the container
  docker exec "$CONTAINER" bash -c \
    "pkill -9 -f 'ros2 launch' 2>/dev/null; true" 2>/dev/null || true
  # Kill any orphaned processes still holding CycloneDDS UDP ports on the host
  # (can accumulate if the container was force-stopped or restarted)
  local pids
  pids=$(ss -ulnp 2>/dev/null \
    | grep -oP '(74|75)[0-9]{2}.*?pid=\K[0-9]+' \
    | sort -u)
  if [ -n "$pids" ]; then
    log "Releasing orphaned DDS ports..."
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null || true
  fi
  # Brief pause so sockets reach TIME_WAIT→CLOSED before the next bind
  sleep 1
}

dexec() {
  local flags="-i"
  [ -t 0 ] && flags="-it"
  docker exec $flags "$CONTAINER" "$@"
}

drun() {
  dexec bash -c "$ROS_ENV $*"
}

# ── Commands ─────────────────────────────────────────────────────────
cmd_shell() {
  ensure_container
  log "Entering container..."
  dexec bash
}

cmd_build() {
  ensure_container
  log "Building workspace..."
  drun "cd /workspace/Morpheus/morpheus_ws && colcon build --symlink-install"
  log "Build complete."
}

cmd_test() {
  ensure_container
  log "Running tests..."
  drun "cd /workspace/Morpheus/morpheus_ws && \
    colcon test --packages-select morpheus_control morpheus_description morpheus_nav2 && \
    colcon test-result --verbose"
}

cmd_launch() {
  ensure_container
  kill_ros
  log "Launching Gazebo + Nav2 + RViz..."
  docker exec -d "$CONTAINER" bash -c "
    $ROS_ENV
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    ros2 launch morpheus_simulation morpheus_spawn.launch.py \
      with_rviz:=true with_teleop:=false \
      >/tmp/morpheus_launch.log 2>&1
  "
  log "Stack starting — follow logs: docker exec $CONTAINER tail -f /tmp/morpheus_launch.log"
}

cmd_clean() {
  log "Stopping container..."
  docker compose down 2>/dev/null || true
  log "Rebuilding image from scratch..."
  HOST_UID="$(id -u)" HOST_GID="$(id -g)" docker compose build --no-cache
  log "Clean rebuild done. Run './docker/run.sh build' to compile workspace."
}

cmd_stop() {
  docker compose down
  log "Container stopped."
}

# ── Main ─────────────────────────────────────────────────────────────
case "${1:-shell}" in
  shell)   cmd_shell ;;
  build)   cmd_build ;;
  test)    cmd_test ;;
  launch)  shift; cmd_launch "$@" ;;
  clean)   cmd_clean ;;
  stop)    cmd_stop ;;
  *)
    echo "Usage: $0 {shell|build|test|launch|clean|stop}"
    echo ""
    echo "  shell   Enter the container (default)"
    echo "  build   colcon build --symlink-install"
    echo "  test    Run all tests"
    echo "  launch  Launch full simulation stack"
    echo "  clean   Wipe build cache and rebuild image"
    echo "  stop    Stop the container"
    exit 1
    ;;
esac
