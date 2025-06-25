#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_gain_node',
            output='screen',
            remappings=[
                ('/imu/data_raw', '/imu/data'),
            ],
            parameters=[{
                'do_bias_estimation': True,
                'do_adaptive_gain': True,
                'use_mag': False,
                'gain_acc': 0.01,
                'gain_mag': 0.01,
                'publish_tf': False,
                'publish_debug_topics': False,
            }],
        ),
    ])
