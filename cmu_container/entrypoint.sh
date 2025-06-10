#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash || true

exec "$@"
