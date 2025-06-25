#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # A multithreaded container to host your composable node
    container = ComposableNodeContainer(
        name='novatel_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='novatel_gps_driver',
                plugin='novatel_gps_driver::NovatelGpsNodelet',
                name='novatel',
                parameters=[{
                    'verbose': False,
                    'wait_for_sync': True,
                    'publish_diagnostics': True,
                    'connection_type': 'serial',
                    'device': '/dev/ttyUSB2',
                    'use_binary_messages': True,
                    'publish_novatel_velocity': True,
                    'frame_id': '/gps',
                    'publish_imu_messages': True,
                    'imu_frame_id': '/imu_link',
                    'imu_rate': 125,
                    'imu_sample_rate': -1,
                }],
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
