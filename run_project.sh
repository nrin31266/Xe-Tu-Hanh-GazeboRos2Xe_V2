#!/bin/bash
set -e

cd ~/ros2_ws || exit 1

export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/humble/setup.bash

if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "ERROR NOT BUILT"
  exit 1
fi

echo "CLEAN OLD GAZEBO"

# Kill theo ten
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f gazebo  2>/dev/null || true

# Kill theo port 11345 (BAT BUOC)
PIDS=$(lsof -t -i:11345 2>/dev/null || true)
if [ -n "$PIDS" ]; then
  echo "PORT 11345 IN USE KILL"
  kill -9 $PIDS 2>/dev/null || true
fi

rm -rf /tmp/gazebo* 2>/dev/null || true
sleep 1

# Kiem tra lan cuoi
PIDS2=$(lsof -t -i:11345 2>/dev/null || true)
if [ -n "$PIDS2" ]; then
  echo "ERROR PORT 11345 STILL BUSY"
  exit 1
fi

launch_file="${1:-autopilot.launch.py}"
echo "RUN LAUNCH $launch_file"
ros2 launch my_robot_controller "$launch_file"
