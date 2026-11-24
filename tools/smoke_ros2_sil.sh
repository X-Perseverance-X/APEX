#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="$ROOT/ros2_ws"
LOG="${TMPDIR:-/tmp}/apex_ros2_sil_smoke.log"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble bulunamadı: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Workspace derlenmemiş. Önce tools/setup_ros2_wsl.sh çalıştır." >&2
  exit 1
fi

# ROS setup scripts normalde nounset kapalı bir shell bekler.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# Kullanıcının açık Gazebo / ROS oturumlarıyla discovery, /clock veya dünya adı
# çakışmasın. Smoke test kendi DDS domain ve Ignition Transport partition'ında.
export ROS_DOMAIN_ID="${APEX_SMOKE_ROS_DOMAIN_ID:-77}"
export ROS2CLI_DISABLE_DAEMON=1
export IGN_PARTITION="${APEX_SMOKE_IGN_PARTITION:-apex_meta1_sil_smoke}"

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 -- "-$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -TERM -- "-$launch_pid" 2>/dev/null || true
      for _ in {1..6}; do
        kill -0 -- "-$launch_pid" 2>/dev/null || break
        sleep 0.5
      done
    fi
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -KILL -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

: > "$LOG"
setsid ros2 launch apex_gazebo sil.launch.py gui:=false >"$LOG" 2>&1 &
launch_pid=$!

deadline=$((SECONDS + 35))
while (( SECONDS < deadline )); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "SIL launch erken kapandı:" >&2
    tail -n 80 "$LOG" >&2
    exit 1
  fi
  if grep -q 'spawner_joint_state_broadcaster.*Configured and activated' "$LOG" &&
     grep -q 'spawner_joint_group_position_controller.*Configured and activated' "$LOG"; then
    break
  fi
  sleep 1
done

if ! grep -q 'spawner_joint_state_broadcaster.*Configured and activated' "$LOG" ||
   ! grep -q 'spawner_joint_group_position_controller.*Configured and activated' "$LOG"; then
  echo "Controller'lar 35 saniyede active olmadı:" >&2
  tail -n 80 "$LOG" >&2
  exit 1
fi

topics="$(timeout 15s ros2 topic list)"
for topic in /clock /joint_states /apex/imu /apex/tof_scan /apex/camera /apex/lidar_scan /apex/gps /apex/odom /apex/target_observation /apex/tracking/status /apex/gait/status /apex/sim/performance; do
  if ! grep -qx "$topic" <<<"$topics"; then
    echo "Eksik topic: $topic" >&2
    tail -n 80 "$LOG" >&2
    exit 1
  fi
done

timeout 10s ros2 topic echo /joint_states --once >/dev/null
timeout 10s ros2 topic echo /apex/imu --once >/dev/null
lidar_frame="$(timeout 10s ros2 topic echo /apex/lidar_scan --once --field header.frame_id | head -1)"
camera_frame="$(timeout 10s ros2 topic echo /apex/camera --once --field header.frame_id | head -1)"
if [[ "$lidar_frame" != "lidar_link" || "$camera_frame" != "camera_optical_frame" ]]; then
  echo "Sensor frame override failed: camera=$camera_frame lidar=$lidar_frame" >&2
  exit 1
fi

echo "[PASS] apex_description + apex_gazebo launch"
echo "[PASS] joint_state_broadcaster active"
echo "[PASS] joint_group_position_controller active"
echo "[PASS] clock, joints, IMU, ToF, camera, LiDAR, GPS, odom ve autonomy topicleri"
echo "[PASS] ROS TF sensor frames: camera_optical_frame, lidar_link"
echo "Log: $LOG"
