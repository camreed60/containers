#!/bin/bash
set -e

id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} ros

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash || true

exec "$@"
