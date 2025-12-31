#!/bin/bash
# Capture filtered Nav2 logs for post-test analysis.
#
# Usage:
#   ./scripts/capture_log.sh              # auto-named by timestamp
#   ./scripts/capture_log.sh my_note      # append a note to filename
#
# Captures: progress checker, recovery, goal success/fail, controller velocity,
#           BT state, stuck detection, waypoint timing.
# Output:   logs/test_<timestamp>[_note].log
#
# Run alongside the simulation; Ctrl+C to stop.

set -euo pipefail

NOTE="${1:-}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/logs"
mkdir -p "$LOG_DIR"

if [ -n "$NOTE" ]; then
  LOG_FILE="$LOG_DIR/test_${TIMESTAMP}_${NOTE// /_}.log"
else
  LOG_FILE="$LOG_DIR/test_${TIMESTAMP}.log"
fi

echo "[capture] Logging to $LOG_FILE"
echo "[capture] Ctrl+C to stop."
echo ""

# Write a header so each log is self-describing
cat > "$LOG_FILE" <<HEADER
# Morpheus Nav2 Test Log
# Date:  $(date)
# Note:  ${NOTE:-none}
# ---
HEADER

# Keywords to capture (case-insensitive)
FILTER="progress\|recovery\|Failed\|Succeeded\|cancel\|timeout\|stuck\|\
Navigating to\|Reached\|Patrol\|lap\|waypoint\|Goal\|\
controller\|velocity\|ClearAnd\|SpinAnd\|BT\|recovery\|\
warn\|error\|ERROR\|WARN"

docker exec morpheus-sim bash -c "tail -f /tmp/morpheus_launch.log 2>/dev/null" \
  | grep --line-buffered -i "$FILTER" \
  | while IFS= read -r line; do
      ts="[$(date '+%H:%M:%S')]"
      echo "$ts $line" | tee -a "$LOG_FILE"
    done
