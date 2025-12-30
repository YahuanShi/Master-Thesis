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
  if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    log "Preparing container..."
    xhost +local:docker >/dev/null 2>&1 || true
    HOST_UID="$(id -u)" HOST_GID="$(id -g)" docker compose up -d --build
    log "Container ready."
  fi
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
  log "Launching full stack..."
  drun "cd /workspace/Morpheus && ./run_morpheus_all.sh $*"
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
