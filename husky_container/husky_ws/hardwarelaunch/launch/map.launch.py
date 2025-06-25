#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    livox_share   = get_package_share_directory('livox_ros_driver')

    mapping_velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping_velodyne.launch.py')
        )
    )

    livox_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch', 'livox_lidar_msg.launch.py')
        )
    )

    return LaunchDescription([
        mapping_velodyne,
        livox_lidar,
    ])
