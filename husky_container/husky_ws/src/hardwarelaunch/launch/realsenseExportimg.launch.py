#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    extract_node = Node(
        package='image_view',
        executable='extract_images',
        name='extract',
        output='screen',
        respawn=False,
        cwd=EnvironmentVariable('ROS_HOME'),
        remappings=[
            ('image', '/camera/color/image_raw'),
        ]
    )

    return LaunchDescription([
        extract_node,
    ])
