#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="$ROOT/ros2_ws"
LOG="${TMPDIR:-/tmp}/apex_autonomy_sil_test.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${APEX_AUTONOMY_ROS_DOMAIN_ID:-78}"
export ROS2CLI_DISABLE_DAEMON=1
export IGN_PARTITION="${APEX_AUTONOMY_IGN_PARTITION:-apex_autonomy_sil_test}"

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

deadline=$((SECONDS + 45))
while (( SECONDS < deadline )); do
  topics="$(timeout 5s ros2 topic list 2>/dev/null || true)"
  if grep -qx /apex/target_observation <<<"$topics" &&
     grep -qx /apex/tracking/status <<<"$topics" &&
     grep -qx /apex/sim/performance <<<"$topics"; then
    break
  fi
  sleep 1
done

observation="$(timeout 15s ros2 topic echo /apex/target_observation --once --field vector)"
confidence="$(awk '/z:/ {print $2; exit}' <<<"$observation")"
python3 -c "assert float('$confidence') >= 0.5, 'SIL target was not detected'"
lidar_frame="$(timeout 8s ros2 topic echo /apex/lidar_scan --once --field header.frame_id | head -1)"
camera_frame="$(timeout 8s ros2 topic echo /apex/camera --once --field header.frame_id | head -1)"

disabled_status="$(timeout 8s ros2 topic echo /apex/tracking/status --once --field data)"
grep -q disabled <<<"$disabled_status"

odom_before="$(timeout 8s ros2 topic echo /apex/odom --once --field pose.pose.position)"
x_before="$(awk '/x:/ {print $2; exit}' <<<"$odom_before")"
y_before="$(awk '/y:/ {print $2; exit}' <<<"$odom_before")"

timeout 8s ros2 service call /apex/tracking/enable std_srvs/srv/SetBool '{data: true}' >/dev/null
sleep 1
command="$(timeout 8s ros2 topic echo /apex/cmd_vel --once)"
linear="$(awk '/linear:/{seen=1; next} seen && /x:/{print $2; exit}' <<<"$command")"
angular="$(awk '/angular:/{seen=1; next} seen && /z:/{print $2; exit}' <<<"$command")"
python3 -c "assert abs(float('$linear')) + abs(float('$angular')) > 0.01, 'tracking command stayed zero'"
sleep 2
odom_after="$(timeout 8s ros2 topic echo /apex/odom --once --field pose.pose.position)"
x_after="$(awk '/x:/ {print $2; exit}' <<<"$odom_after")"
y_after="$(awk '/y:/ {print $2; exit}' <<<"$odom_after")"
travel="$(python3 -c "import math; print(math.hypot(float('$x_after')-float('$x_before'), float('$y_after')-float('$y_before')))")"
python3 -c "assert float('$travel') > 0.05, 'cmd_vel did not move the SIL body'"

performance="$(timeout 12s ros2 topic echo /apex/sim/performance --once --field data)"
grep -q camera_fps <<<"$performance"

timeout 8s ros2 service call /apex/tracking/enable std_srvs/srv/SetBool '{data: false}' >/dev/null

echo "[PASS] magenta SIL target detected, confidence=$confidence"
echo "[INFO] sensor frames: camera=$camera_frame lidar=$lidar_frame"
echo "[PASS] tracking is safety-disabled at startup"
echo "[PASS] enabled controller produced cmd_vel: linear=$linear angular=$angular"
echo "[PASS] physical tripod contact gait moved body by ${travel}m"
echo "[PASS] performance telemetry: $performance"
echo "Log: $LOG"
