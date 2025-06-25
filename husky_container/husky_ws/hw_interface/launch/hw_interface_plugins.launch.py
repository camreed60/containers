#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Locate the Novatel SPAN plugin package
    plugin_share = get_package_share_directory('hw_interface_plugin_novatel_span')

    # Include its ROS 2–ported launch
    novatel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plugin_share, 'launch', 'Novatel_Span_Serial_Interface.launch.py')
        )
    )

    return LaunchDescription([
        novatel_launch,
    ])
