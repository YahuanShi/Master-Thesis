#!/bin/bash
# =============================================
#  Morpheus demo recorder
#  Records Gazebo + RViz side-by-side, runs a scripted driving
#  sequence and patrol mission, then converts to GIF for README.
#
#  Runs on the HOST. Uses docker/run.sh to manage the container
#  (inherits docker-compose GPU, DRI, FastDDS config). ffmpeg
#  captures the X11 display from outside.
#
#  Usage:
#    ./scripts/record_demo.sh              # full auto
#    ./scripts/record_demo.sh --skip-build # skip colcon build
#    ./scripts/record_demo.sh --manual     # launch stack, record, let you drive
# =============================================
set -euo pipefail

# ── Configuration ────────────────────────────────────────────────────
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_DIR="$PROJECT_DIR/docker"
CONTAINER="morpheus-sim"
DISPLAY_ID="${DISPLAY:-:0.0}"
FPS=30
OUTPUT_DIR="$PROJECT_DIR/docs"
RAW_VIDEO="$OUTPUT_DIR/demo_raw.mp4"
FINAL_MP4="$OUTPUT_DIR/demo.mp4"
FINAL_GIF="$OUTPUT_DIR/demo.gif"
GIF_WIDTH=800
GIF_FPS=10

SKIP_BUILD=false
MANUAL_MODE=false

for arg in "$@"; do
  case "$arg" in
    --skip-build) SKIP_BUILD=true ;;
    --manual)     MANUAL_MODE=true ;;
  esac
done

# ── Helpers ──────────────────────────────────────────────────────────
log()  { echo -e "\033[1;36m[demo]\033[0m $*"; }
die()  { echo -e "\033[1;31m[demo]\033[0m $*" >&2; exit 1; }

ROS_ENV="source /opt/ros/humble/setup.bash && \
  [ -f /tmp/slam_toolbox_src/install/setup.bash ] && source /tmp/slam_toolbox_src/install/setup.bash; \
  [ -f /workspace/Morpheus/morpheus_ws/install/setup.bash ] && source /workspace/Morpheus/morpheus_ws/install/setup.bash;"

drun() {
  local flags="-i"
  [ -t 0 ] && flags="-it"
  docker exec $flags "$CONTAINER" bash -c "$ROS_ENV $*"
}

cleanup() {
  log "Cleaning up..."
  [ -n "${FFMPEG_PID:-}" ] && kill "$FFMPEG_PID" 2>/dev/null && wait "$FFMPEG_PID" 2>/dev/null
  docker exec "$CONTAINER" bash -c "pkill -f 'ros2 launch|morpheus_spawn|ign gazebo|gz sim' 2>/dev/null" || true
  log "Done."
}
trap cleanup EXIT

# ── Preflight checks ────────────────────────────────────────────────
command -v ffmpeg  >/dev/null || die "ffmpeg not found. Install it first."
command -v docker  >/dev/null || die "docker not found."

RESOLUTION=$(xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2}')
[ -z "$RESOLUTION" ] && die "Cannot detect screen resolution. Is X11 running?"
log "Display $DISPLAY_ID @ $RESOLUTION"

mkdir -p "$OUTPUT_DIR"

# ── Step 1: Ensure container via docker-compose ─────────────────────
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  log "Starting container via docker-compose..."
  rm -f /dev/shm/fastrtps_* 2>/dev/null || true
  xhost +local:docker >/dev/null 2>&1 || true
  cd "$DOCKER_DIR"
  HOST_UID="$(id -u)" HOST_GID="$(id -g)" docker compose up -d --build
  cd "$PROJECT_DIR"
  sleep 2
  log "Container started."
else
  log "Container already running."
fi

# ── Step 2: Build workspace ─────────────────────────────────────────
if [ "$SKIP_BUILD" = false ]; then
  log "Building workspace inside container..."
  drun "cd /workspace/Morpheus/morpheus_ws && colcon build --symlink-install 2>&1 | tail -5"
  log "Build complete."
else
  log "Skipping build (--skip-build)."
fi

# ── Step 3: Start ffmpeg recording ───────────────────────────────────
log "Starting screen recording → $RAW_VIDEO"
ffmpeg -y -f x11grab -framerate "$FPS" -video_size "$RESOLUTION" \
  -i "$DISPLAY_ID" -c:v libx264 -preset fast -crf 18 \
  "$RAW_VIDEO" </dev/null >/dev/null 2>&1 &
FFMPEG_PID=$!
sleep 1

if ! kill -0 "$FFMPEG_PID" 2>/dev/null; then
  die "ffmpeg failed to start. Check X11 and display settings."
fi
log "Recording (PID $FFMPEG_PID)."

# ── Step 4: Launch full stack ────────────────────────────────────────
# Kill any existing stack first — a second simultaneous launch duplicates TF
# publishers, breaks CycloneDDS participant indices, and causes RViz red errors.
log "Stopping any running ROS stack..."
docker exec "$CONTAINER" bash -c "pkill -9 -f 'ros2 launch' 2>/dev/null; true"
# Release any orphaned DDS ports on the host (accumulated if container was
# force-restarted while using the old setsid/nohup launch approach)
ss -ulnp 2>/dev/null | grep -oP '(74|75)[0-9]{2}.*?pid=\K[0-9]+' | sort -u | \
  xargs kill -9 2>/dev/null || true
sleep 1

log "Launching Gazebo + Nav2 + RViz..."
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/Morpheus/morpheus_ws/install/setup.bash &&
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json &&
  ros2 launch morpheus_simulation morpheus_spawn.launch.py \
    with_rviz:=true with_teleop:=false \
    >/tmp/morpheus_launch.log 2>&1
"

# ── Step 5: Wait for system ready ───────────────────────────────────
log "Waiting for Gazebo and Nav2 to initialize..."

wait_for_nav2() {
  local timeout="$1" elapsed=0
  while [ $elapsed -lt "$timeout" ]; do
    if docker exec "$CONTAINER" bash -c \
        "grep -q 'Managed nodes are active' /tmp/morpheus_launch.log 2>/dev/null"; then
      return 0
    fi
    sleep 5
    elapsed=$((elapsed + 5))
  done
  return 1
}

if ! wait_for_nav2 240; then
  die "Timed out waiting for Nav2 to become active."
fi
log "Nav2 active."

# Extra settle time for Gazebo to finish rendering
sleep 5
log "System ready."

if [ "$MANUAL_MODE" = true ]; then
  # ── Manual mode: let user drive ──────────────────────────────────
  log "=========================================="
  log "  MANUAL MODE - Stack is running."
  log "  Drive via: ros2 topic pub /cmd_vel ..."
  log "  Press Enter here to stop recording."
  log "=========================================="
  read -r
else
  # ── Step 6: Scripted driving demo ──────────────────────────────────
  log "Running scripted driving sequence..."

  pub_cmd() {
    local lx="$1" ly="$2" az="$3" dur="$4"
    drun "timeout $dur ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
      \"{linear: {x: $lx, y: $ly}, angular: {z: $az}}\" \
      --rate 10 >/dev/null 2>&1" || true
  }

  # Scene overview (let camera settle)
  sleep 3

  # Forward drive
  log "  Forward..."
  pub_cmd 0.8 0.0 0.0 4

  sleep 1

  # Ackermann turn
  log "  Ackermann turn..."
  pub_cmd 0.6 0.0 0.5 4

  sleep 1

  # Crab walk
  log "  Crab walk..."
  pub_cmd 0.0 0.5 0.0 3

  sleep 1

  # Pivot turn
  log "  Pivot turn..."
  pub_cmd 0.0 0.0 0.8 3

  sleep 1

  # Stop
  pub_cmd 0.0 0.0 0.0 1

  # ── Step 7: Patrol mission ───────────────────────────────────────
  log "Starting patrol mission..."
  drun "timeout 60 ros2 run morpheus_nav2 patrol_mission.py \
    >/dev/null 2>&1" || true
  log "Patrol mission finished."

  sleep 3
fi

# ── Step 8: Stop recording ──────────────────────────────────────────
log "Stopping recording..."
kill "$FFMPEG_PID" 2>/dev/null
wait "$FFMPEG_PID" 2>/dev/null || true
unset FFMPEG_PID

if [ ! -f "$RAW_VIDEO" ]; then
  die "Recording file not found: $RAW_VIDEO"
fi

RAW_SIZE=$(du -h "$RAW_VIDEO" | cut -f1)
log "Raw recording: $RAW_VIDEO ($RAW_SIZE)"

# ── Step 9: Post-process ────────────────────────────────────────────
log "Encoding final MP4..."
ffmpeg -y -i "$RAW_VIDEO" \
  -vf "scale=1920:-2" \
  -c:v libx264 -preset slow -crf 22 \
  "$FINAL_MP4" </dev/null 2>/dev/null

log "Generating GIF (${GIF_WIDTH}px, ${GIF_FPS}fps)..."
ffmpeg -y -i "$FINAL_MP4" \
  -vf "fps=${GIF_FPS},scale=${GIF_WIDTH}:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer" \
  "$FINAL_GIF" </dev/null 2>/dev/null

FINAL_MP4_SIZE=$(du -h "$FINAL_MP4" | cut -f1)
GIF_SIZE=$(du -h "$FINAL_GIF" | cut -f1)

log "=========================================="
log "  Recording complete!"
log "  Raw:  $RAW_VIDEO ($RAW_SIZE)"
log "  MP4:  $FINAL_MP4 ($FINAL_MP4_SIZE)"
log "  GIF:  $FINAL_GIF ($GIF_SIZE)"
log "=========================================="
log ""
log "To trim the video before converting to GIF:"
log "  ffmpeg -i $FINAL_MP4 -ss 00:00:05 -to 00:00:35 -c copy trimmed.mp4"
log "  Then re-run the GIF command on trimmed.mp4"
log ""
log "To use in README, uncomment the demo line:"
log '  ![Morpheus Demo](docs/demo.gif)'
