#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('hw_interface_plugin_novatel_span')
    params_file = PathJoinSubstitution([
        pkg, 'launch', 'launch_params',
        'Novatel_Span_Serial_launch_params.yaml'
    ])

    novatel_node = Node(
        package='hw_interface_plugin_novatel_span',
        executable='novatel_span_serial',      # your plugin executable
        name='novatel_span_serial',
        namespace='hw_interface',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([novatel_node])
