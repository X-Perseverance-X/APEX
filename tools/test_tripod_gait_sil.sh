#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="$ROOT/ros2_ws"
LOG="${TMPDIR:-/tmp}/apex_tripod_gait_sil.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${APEX_GAIT_ROS_DOMAIN_ID:-79}"
export ROS2CLI_DISABLE_DAEMON=1
export IGN_PARTITION="${APEX_GAIT_IGN_PARTITION:-apex_tripod_gait_test}"

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 -- "-$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

: > "$LOG"
setsid ros2 launch apex_gazebo sil.launch.py gui:=false profile:=fast >"$LOG" 2>&1 &
launch_pid=$!
python3 "$SCRIPT_DIR/tripod_gait_probe.py"
echo "Log: $LOG"
