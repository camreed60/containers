#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hw_if_share = get_package_share_directory('hw_interface')

    # Include all plugin launches
    plugins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_if_share, 'launch', 'hw_interface_plugins.launch.py')
        )
    )

    # Launch the core hw_interface node
    hw_interface_node = Node(
        package='hw_interface',
        executable='hw_interface_node',
        name='hw_interface',
        namespace='hw_interface',
        output='screen',
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        plugins_launch,
        hw_interface_node,
    ])
