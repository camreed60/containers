#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # 1) Declare all launch arguments (with your ROS1 defaults)
    args = [
        DeclareLaunchArgument('camera_name',   default_value='zed',  description='The name you want'),
        DeclareLaunchArgument('camera_model',  default_value='zed',  description="'zed'|'zedm'|'zed2'"),
        DeclareLaunchArgument('node_name',     default_value='zed_node'),
        DeclareLaunchArgument('svo_file',      default_value='',     description='Path to SVO file'),
        DeclareLaunchArgument('stream',        default_value='',     description='Remote stream IP:port'),
        DeclareLaunchArgument('base_frame',    default_value='zed_link'),
        DeclareLaunchArgument('publish_urdf',  default_value='true', description='Publish ZED urdf?'),
        DeclareLaunchArgument('camera_id',     default_value='0'),
        DeclareLaunchArgument('gpu_id',        default_value='-1'),
        DeclareLaunchArgument('cam_pos_x',     default_value='0.0'),
        DeclareLaunchArgument('cam_pos_y',     default_value='0.0'),
        DeclareLaunchArgument('cam_pos_z',     default_value='0.0'),
        DeclareLaunchArgument('cam_roll',      default_value='0.0'),
        DeclareLaunchArgument('cam_pitch',     default_value='0.0'),
        DeclareLaunchArgument('cam_yaw',       default_value='0.0'),
    ]

    # 2) Build the xacro command to generate the URDF string
    zed_pkg = get_package_share_directory('zed_wrapper')
    xacro_file = os.path.join(zed_pkg, 'urdf', 'zed_descr.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro', ' ', xacro_file,
            ' camera_name:=', LaunchConfiguration('camera_name'),
            ' camera_model:=', LaunchConfiguration('camera_model'),
            ' base_frame:=',     LaunchConfiguration('base_frame'),
            ' cam_pos_x:=',      LaunchConfiguration('cam_pos_x'),
            ' cam_pos_y:=',      LaunchConfiguration('cam_pos_y'),
            ' cam_pos_z:=',      LaunchConfiguration('cam_pos_z'),
            ' cam_roll:=',       LaunchConfiguration('cam_roll'),
            ' cam_pitch:=',      LaunchConfiguration('cam_pitch'),
            ' cam_yaw:=',        LaunchConfiguration('cam_yaw'),
        ]),
        value_type=str,
    )

    # 3) robot_state_publisher group (conditionally run if publish_urdf==true)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=[LaunchConfiguration('camera_name'), '_state_publisher'],
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    rsp_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('publish_urdf')),
        actions=[rsp_node],
    )

    # 4) ZED wrapper node, loading the two ROS1-style YAMLs plus inline overrides
    common_yaml = os.path.join(zed_pkg, 'params', 'common.yaml')
    model_yaml  = os.path.join(zed_pkg, 'params',
                   [LaunchConfiguration('camera_model'), '.yaml'])
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            common_yaml,
            model_yaml,
            # inline overrides exactly matching your <param> tags:
            {'general.camera_name':    LaunchConfiguration('camera_name')},
            {'general.base_frame':     LaunchConfiguration('base_frame')},
            {'svo_file':               LaunchConfiguration('svo_file')},
            {'stream':                 LaunchConfiguration('stream')},
            {'general.zed_id':         LaunchConfiguration('camera_id')},
            {'general.gpu_id':         LaunchConfiguration('gpu_id')},
        ],
    )

    return LaunchDescription([
        *args,
        rsp_group,
        zed_node,
    ])
