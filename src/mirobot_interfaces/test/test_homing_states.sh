#!/usr/bin/env bash
# The G-code writer's homed state against a fake firmware on a pty (socat): never a real serial port.
# Needs the workspace built (install/) and socat. Isolated: ROS domain 97, localhost only.
#   bash src/mirobot_interfaces/test/test_homing_states.sh
#
# Regression (13 Nov 2026): firmware in Hold at connect, $H never ran, the writer reported "Robot homed
# (firmware state Hold)" after the homing timeout and published joint states of all zeros.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(cd "$HERE/../../.." && pwd)}"
command -v socat >/dev/null || { echo "test_homing_states: socat missing (apt install socat)"; exit 2; }
set +u; . /opt/ros/jazzy/setup.bash; . "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
TMP="$(mktemp -d /tmp/mirobot-homing-test.XXXXXX)"
FAIL=0

# run <name> <seconds> <homing true|false> <fake firmware args...>: leaves logs in $TMP/<name>
run() {
  local name=$1 secs=$2 homing=$3; shift 3
  local d="$TMP/$name"; mkdir -p "$d"
  socat pty,raw,echo=0,link="$d/robot" pty,raw,echo=0,link="$d/fw" 2>"$d/socat.log" &
  local socat=$!; sleep 1
  python3 "$HERE/fake_firmware_pty.py" "$d/fw" "$@" >"$d/fw.log" 2>&1 &
  local fw=$!
  ros2 topic echo /tw/joint_states sensor_msgs/msg/JointState --field position >"$d/js.log" 2>&1 &
  local js=$!
  ros2 topic echo /tw/hw_status diagnostic_msgs/msg/DiagnosticStatus >"$d/hw.log" 2>&1 &
  local hw=$!
  timeout "$secs" ros2 run mirobot_interfaces mirobot_gcode_writer --ros-args \
    -p robot.name:=tw -p topics.prefix:=tw -p topics.mirobot_status:=hw_status \
    -p serial.port:="$d/robot" -p robot.perform_homing_on_startup:="$homing" -p robot.homing_timeout_seconds:=8 \
    >"$d/writer.log" 2>&1
  kill $fw $js $hw $socat 2>/dev/null; wait $fw $js $hw $socat 2>/dev/null
}
js_count() { grep -c '^---' "$TMP/$1/js.log"; }
last_kv() { grep -A1 "key: $2\$" "$TMP/$1/hw.log" | grep value | tail -1 | sed "s/.*value: //; s/'//g"; }
check() {
  if [ "$2" = "$3" ]; then echo "PASS $1"; else echo "FAIL $1 (got '$2', expected '$3')"; FAIL=1; fi
}

run hold_homing 20 true --start Hold --home ignore
check "Hold at connect, homing never runs: no joint states" "$(js_count hold_homing)" 0
check "  ... status error" "$(last_kv hold_homing state)" error
check "  ... not homed" "$(last_kv hold_homing is_homed)" false
check "  ... the error says why" "$(last_kv hold_homing error_message | cut -c1-27)" "Homing did not complete wit"

run alarm_homing 12 true --start Alarm --home ok
check "Alarm, homing completes: joint states published" "$( [ "$(js_count alarm_homing)" -gt 5 ] && echo yes)" yes
check "  ... idle and homed" "$(last_kv alarm_homing state)/$(last_kv alarm_homing is_homed)" idle/true

run idle_no_homing 8 false --start Idle
check "Idle, connect without homing: joint states published" "$( [ "$(js_count idle_no_homing)" -gt 5 ] && echo yes)" yes

run hold_no_homing 8 false --start Hold
check "Hold, connect without homing: not homed, no joint states" "$(last_kv hold_no_homing is_homed)/$(js_count hold_no_homing)" false/0

echo "logs in $TMP"
exit $FAIL
