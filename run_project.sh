#!/bin/bash

cd ~/ros2_ws || exit 1

# ít log khi source
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/humble/setup.bash

if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "Chưa build. Chạy ./clean_build.sh trước."
  exit 1
fi

launch_file="${1:-autopilot.launch.py}"
echo "Run: ros2 launch my_robot_controller $launch_file"
ros2 launch my_robot_controller "$launch_file"
