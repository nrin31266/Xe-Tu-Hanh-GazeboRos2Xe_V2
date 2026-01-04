#!/usr/bin/env bash
set -euo pipefail

missing=0
warnings=0

ok() { printf "OK: %s\n" "$*"; }
warn() { printf "WARN: %s\n" "$*"; warnings=1; }
fail() { printf "MISSING: %s\n" "$*"; missing=1; }

check_cmd() {
  local cmd="$1"
  if command -v "$cmd" >/dev/null 2>&1; then
    ok "command '$cmd' found"
  else
    fail "command '$cmd' not found"
  fi
}

check_dir() {
  local path="$1"
  if [ -d "$path" ]; then
    ok "directory exists: $path"
  else
    fail "directory missing: $path"
  fi
}

check_file() {
  local path="$1"
  if [ -f "$path" ]; then
    ok "file exists: $path"
  else
    fail "file missing: $path"
  fi
}

echo "== Core dependencies =="

if [ -d /opt/ros/humble ]; then
  ok "ROS 2 Humble installed at /opt/ros/humble"
else
  fail "ROS 2 Humble not found at /opt/ros/humble"
fi

if [ "${ROS_DISTRO:-}" = "humble" ]; then
  ok "ROS_DISTRO=humble"
else
  warn "ROS_DISTRO is not 'humble' (source /opt/ros/humble/setup.bash)"
fi

if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 CLI is on PATH"
else
  warn "ros2 CLI not on PATH (source /opt/ros/humble/setup.bash)"
fi

check_cmd python3
check_cmd colcon

check_cmd gazebo

if [ -d /opt/ros/humble/share/gazebo_ros ]; then
  ok "gazebo_ros package found"
else
  fail "gazebo_ros package not found in /opt/ros/humble/share"
fi

if [ -d /opt/ros/humble/share/turtlebot3_gazebo ]; then
  ok "turtlebot3_gazebo package found"
else
  fail "turtlebot3_gazebo package not found in /opt/ros/humble/share"
fi

echo
echo "== Project assets =="

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
check_dir "$repo_root/src/my_car_description/models"
check_dir "$repo_root/src/my_world/worlds"
check_file "$repo_root/src/my_world/worlds/highway.world"

launch_file="$repo_root/src/my_robot_controller/launch/autopilot.launch.py"
if [ -f "$launch_file" ] && grep -q "/home/nguyentien/ros2_ws" "$launch_file"; then
  warn "autopilot.launch.py has a hard-coded path: /home/nguyentien/ros2_ws (update to your workspace)"
fi

echo
if [ "$missing" -ne 0 ]; then
  echo "Result: missing dependencies detected."
  exit 1
fi

if [ "$warnings" -ne 0 ]; then
  echo "Result: warnings detected (review above)."
else
  echo "Result: all checks passed."
fi
