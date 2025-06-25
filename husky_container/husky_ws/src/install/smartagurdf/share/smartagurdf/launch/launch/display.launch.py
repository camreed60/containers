#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # 1) Declare the unused 'model' arg for parity
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='(Unused) model argument'
    )

    # 2) Build robot_description from your Xacro
    desc_pkg = get_package_share_directory('husky_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'husky.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 3) joint_state_publisher_gui node
    js_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 4) robot_state_publisher node (loads robot_description)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 5) RViz2 with the same config file
    smart_pkg = get_package_share_directory('smartagurdf')
    rviz_config = PathJoinSubstitution([smart_pkg, 'urdf.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        model_arg,
        js_pub,
        rsp,
        rviz,
    ])
