#!/usr/bin/env bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# If we have built a ROS 2 workspace under /ros2_ws, source it too
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# 3) Finally, run whatever command was passed in
exec "$@"
