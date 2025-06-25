#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    hw_launch_share = get_package_share_directory('hardwarelaunch')
    husky_base_share = get_package_share_directory('husky_base')
    hw_if_share = get_package_share_directory('hw_interface')
    imu_cf_share = get_package_share_directory('imu_complementary_filter')

    # 1) Serial‐port argument
    serial_arg = DeclareLaunchArgument(
        'husky_serial_port',
        default_value='/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.2:1.0-port0',
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
            'max_acceleration': 0.1,
            'max_speed': 0.80,
            'wheel_diameter': 0.3302,
            'polling_timeout': 10.0,
        }],
        output='screen'
    )

    # 3) Includes (assumes you've ported these to .launch.py already)
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
    zed2i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_launch_share, 'launch', 'zed2i.launch.py')
        )
    )
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_launch_share, 'launch', 'velodyne.launch.py')
        )
    )
    hw_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_if_share, 'launch', 'hw_interface.launch.py')
        )
    )
    comp_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_cf_share, 'launch', 'complementary_filter.launch.py')
        )
    )

    # 4) Diagnostic Aggregator node
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
        zed2i_launch,
        velodyne_launch,
        hw_interface_launch,
        comp_filter_launch,
        diag_node,
    ])
