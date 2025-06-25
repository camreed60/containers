#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate package shares
    gazebo_share   = get_package_share_directory('gazebo_ros')
    urdf_pkg_share = get_package_share_directory('smartagurdf')

    # 1) Empty Gazebo world
    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'empty_world.launch.py')
        )
    )

    # 2) Static transform from base_link → base_footprint
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40'],
        output='screen'
    )

    # 3) Spawn your URDF into Gazebo
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-file', os.path.join(urdf_pkg_share, 'urdf', 'smartagurdf.urdf'),
            '-entity', 'smartagurdf'
        ],
        output='screen'
    )

    # 4) Fake joint calibration publisher
    fake_calib = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/calibrated',
            'std_msgs/msg/Bool', 'true'
        ],
        output='screen'
    )

    return LaunchDescription([
        empty_world,
        static_tf,
        spawn_model,
        fake_calib,
    ])
