#!/usr/bin/env bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the install space
source /ros2_ws/install/setup.bash

# Run the command passed to the container
exec "$@"
