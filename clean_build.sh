#!/bin/bash
cd ~/ros2_ws || exit 1

echo "Dọn build / install / log..."
rm -rf build install log

echo "Dọn __pycache__ và *.pyc..."
find src -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null
find src -name "*.pyc" -delete 2>/dev/null

echo "BUILD WORKSPACE"

# tắt trace và tránh lỗi nounset
export AMENT_TRACE_SETUP_FILES=0
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/humble/setup.bash

colcon build --symlink-install

echo "HOÀN TẤT"

