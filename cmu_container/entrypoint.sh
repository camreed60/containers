#!/usr/bin/env bash
set -e

# Ensure /ros2_ws/src exists
# CHANGE THIS POINT TO YOUR WS
if [ ! -d "/ros2_ws/src" ]; then
  echo "[entrypoint] '/ros2_ws/src' not found. Creating an empty ROS 2 workspace at /ros2_ws/src..."
  mkdir -p /ros2_ws/src
fi

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the install space
source /ros2_ws/install.setup.bash

# Run the command passed to the container
exec "$@"
