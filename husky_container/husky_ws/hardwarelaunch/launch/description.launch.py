#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    Command,
    PathJoinSubstitution,
)
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the same args as in your ROS 1 XML
    robot_ns_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='/',
        description='Top-level namespace for the robot'
    )
    laser_enabled_arg = DeclareLaunchArgument(
        'laser_enabled',
        default_value=EnvironmentVariable('HUSKY_LMS1XX_ENABLED', default_value='false'),
        description='Enable LMS1XX laser'
    )
    realsense_enabled_arg = DeclareLaunchArgument(
        'realsense_enabled',
        default_value=EnvironmentVariable('HUSKY_REALSENSE_ENABLED', default_value='false'),
        description='Enable Realsense camera'
    )
    urdf_extras_arg = DeclareLaunchArgument(
        'urdf_extras',
        default_value=EnvironmentVariable('HUSKY_URDF_EXTRAS', default_value=''),
        description='Extra URDF snippets'
    )

    # Generate the robot_description parameter via xacro
    pkg_share = get_package_share_directory('smartagurdf')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'smartagurdf.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )

    # In ROS 2 you attach params to a Node—here we publish robot_description
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_ns_arg,
        laser_enabled_arg,
        realsense_enabled_arg,
        urdf_extras_arg,
        rsp_node,
    ])
