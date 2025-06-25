#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    extract_node = Node(
        package='image_view',
        executable='extract_images',
        name='extract',
        respawn=False,
        output='screen',
        cwd=EnvironmentVariable('ROS_HOME'),
        remappings=[
            ('image', '/zed2i/zed_node/left/image_rect_color'),
        ],
    )

    return LaunchDescription([
        extract_node,
    ])
