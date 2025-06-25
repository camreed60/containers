#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hardwarelaunch')
    teleop_cfg = PathJoinSubstitution([pkg_share, 'config', 'teleop.yaml'])

    # 1) Declare the same launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    joystick_arg = DeclareLaunchArgument(
        'joystick',
        default_value='true',
        description='Enable joystick (unused)'
    )

    # 2) joy_node under namespace "joy_teleop", loading teleop.yaml and setting dev param
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace='joy_teleop',
        parameters=[
            teleop_cfg,
            {'dev': LaunchConfiguration('joy_dev')}
        ],
        output='screen'
    )

    # 3) teleop_twist_joy node under same namespace, loading teleop.yaml
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        namespace='joy_teleop',
        parameters=[teleop_cfg],
        output='screen'
    )

    return LaunchDescription([
        joy_dev_arg,
        joystick_arg,
        joy_node,
        teleop_node,
    ])
