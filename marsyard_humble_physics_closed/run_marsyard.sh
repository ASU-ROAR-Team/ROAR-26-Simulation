#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [ ! -d install ]; then
  colcon build --symlink-install
fi
source install/setup.bash
ros2 launch marsyard marsyard.launch.py
