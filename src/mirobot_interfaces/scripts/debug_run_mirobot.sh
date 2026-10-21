#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  set +u
  source "$ROOT_DIR/install/setup.bash"
  set -u
fi

PREFIX="${1:-mirobot_01}"
MODE="${2:-linear}"
SPEED="${3:-1200.0}"
POINT_SLEEP="${4:-2.0}"

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$ROOT_DIR/log/debug_run_$STAMP"
mkdir -p "$RUN_DIR"

RESP_TOPIC="/${PREFIX}/mirobot_response"
STATUS_TOPIC="/${PREFIX}/status"
POSE_TOPIC="/${PREFIX}/target_pose"
MODE_TOPIC="/${PREFIX}/cartesian_mode"
SPEED_TOPIC="/${PREFIX}/cartesian_speed"

echo "Run folder: $RUN_DIR"
echo "Prefix: $PREFIX"
echo "Mode: $MODE"
echo "Speed: $SPEED"
echo "Point sleep: ${POINT_SLEEP}s"

cleanup() {
  if [[ -n "${RESP_PID:-}" ]]; then kill "$RESP_PID" 2>/dev/null || true; fi
  if [[ -n "${STATUS_PID:-}" ]]; then kill "$STATUS_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

stdbuf -oL ros2 topic echo "$RESP_TOPIC" > "$RUN_DIR/mirobot_response.log" 2>&1 &
RESP_PID=$!
stdbuf -oL ros2 topic echo "$STATUS_TOPIC" > "$RUN_DIR/mirobot_status.log" 2>&1 &
STATUS_PID=$!

sleep 1

ros2 topic pub --once "$MODE_TOPIC" std_msgs/msg/String "{data: $MODE}" | tee -a "$RUN_DIR/commands.log"
ros2 topic pub --once "$SPEED_TOPIC" std_msgs/msg/Float64 "{data: $SPEED}" | tee -a "$RUN_DIR/commands.log"

declare -a POINTS=(
  "200 0 230"
  "220 0 230"
  "180 0 230"
  "200 60 230"
  "200 -60 230"
  "200 0 200"
  "200 0 260"
  "230 60 220"
  "230 -60 220"
  "170 60 220"
  "170 -60 220"
  "210 0 180"
)

publish_pose() {
  local pass_name="$1"
  local point_id="$2"
  local x_mm="$3"
  local y_mm="$4"
  local z_mm="$5"
  local qx="$6"
  local qy="$7"
  local qz="$8"
  local qw="$9"

  local x y z
  x="$(awk "BEGIN {printf \"%.4f\", $x_mm/1000.0}")"
  y="$(awk "BEGIN {printf \"%.4f\", $y_mm/1000.0}")"
  z="$(awk "BEGIN {printf \"%.4f\", $z_mm/1000.0}")"

  local marker="[$(date +%H:%M:%S)] PASS=$pass_name POINT=$point_id XYZ_MM=($x_mm,$y_mm,$z_mm) Q=($qx,$qy,$qz,$qw)"
  echo "$marker" | tee -a "$RUN_DIR/commands.log"

  ros2 topic pub --once "$POSE_TOPIC" geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'base_link'}, pose: {position: {x: $x, y: $y, z: $z}, orientation: {x: $qx, y: $qy, z: $qz, w: $qw}}}" \
    >> "$RUN_DIR/commands.log" 2>&1

  sleep "$POINT_SLEEP"
}

run_pass() {
  local pass_name="$1"
  local qx="$2"
  local qy="$3"
  local qz="$4"
  local qw="$5"

  echo "===== PASS $pass_name =====" | tee -a "$RUN_DIR/commands.log"

  local i=1
  for p in "${POINTS[@]}"; do
    read -r x_mm y_mm z_mm <<< "$p"
    publish_pose "$pass_name" "$i" "$x_mm" "$y_mm" "$z_mm" "$qx" "$qy" "$qz" "$qw"
    ((i++))
  done
}

run_pass "RPY_0_0_0" 0.0 0.0 0.0 1.0
run_pass "RPY_0_20_0" 0.0 0.173648 0.0 0.984808
run_pass "RPY_0_-20_0" 0.0 -0.173648 0.0 0.984808
run_pass "RPY_20_0_0" 0.173648 0.0 0.0 0.984808
run_pass "RPY_0_0_45" 0.0 0.0 0.382683 0.923880

sleep 2

echo "Finished debug run. Logs written to: $RUN_DIR"
echo "  - $RUN_DIR/commands.log"
echo "  - $RUN_DIR/mirobot_response.log"
echo "  - $RUN_DIR/mirobot_status.log"