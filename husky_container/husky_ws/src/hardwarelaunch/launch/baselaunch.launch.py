#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1) Serial‐port argument (default matches your XML)
    serial_arg = DeclareLaunchArgument(
        'husky_serial_port',
        default_value='/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0',
        description='Serial port for the Husky base driver'
    )

    # 2) Husky hardware driver node
    husky_node = Node(
        package='husky_base',
        executable='husky_node',
        name='husky_node',
        parameters=[{
            'port': LaunchConfiguration('husky_serial_port'),
            'control_frequency': 10.0,
            'diagnostic_frequency': 1.0,
            'max_acceleration': 0.6,
            'max_speed': 1.0,
            'wheel_diameter': 0.3302,
            'polling_timeout': 10.0,
        }],
        output='screen'
    )

    # 3) Include control & teleop launches (must be ported to .launch.py)
    hw_launch_share = get_package_share_directory('hardwarelaunch')
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_launch_share, 'launch', 'control.launch.py')
        )
    )
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_launch_share, 'launch', 'teleop.launch.py')
        )
    )

    # 4) Diagnostic Aggregator node
    husky_base_share = get_package_share_directory('husky_base')
    diag_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        parameters=[os.path.join(husky_base_share, 'config', 'diagnostics.yaml')],
        output='screen'
    )

    return LaunchDescription([
        serial_arg,
        husky_node,
        control_launch,
        teleop_launch,
        diag_node,
    ])
