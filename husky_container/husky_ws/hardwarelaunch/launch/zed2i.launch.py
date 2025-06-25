#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    hw_share = get_package_share_directory('hardwarelaunch')

    # Declare launch arguments
    args = [
        DeclareLaunchArgument('svo_file',    default_value='', description='SVO file path'),
        DeclareLaunchArgument('stream',      default_value='', description='Stream URI'),
        DeclareLaunchArgument('node_name',   default_value='zed_node', description='ZED node name'),
        DeclareLaunchArgument('camera_model',default_value='zed2i',    description='ZED model'),
        DeclareLaunchArgument('publish_urdf',default_value='true',     description='Publish URDF?'),
        DeclareLaunchArgument('camera_name', default_value='zed2i',    description='Namespace / camera name'),
        DeclareLaunchArgument('base_frame',  default_value='base_link',description='Base frame'),
        DeclareLaunchArgument('cam_pos_x',   default_value='0.33638',  description='Camera X offset'),
        DeclareLaunchArgument('cam_pos_y',   default_value='0.002',    description='Camera Y offset'),
        DeclareLaunchArgument('cam_pos_z',   default_value='0.314',    description='Camera Z offset'),
        DeclareLaunchArgument('cam_roll',    default_value='0.0',      description='Camera roll'),
        DeclareLaunchArgument('cam_pitch',   default_value='0.04',     description='Camera pitch'),
        DeclareLaunchArgument('cam_yaw',     default_value='0.0',      description='Camera yaw'),
    ]

    # Include the existing launch XML under the camera namespace
    zed_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(hw_share, 'config', 'zed_camera.launch.xml')
        ),
        launch_arguments={
            'camera_name':    LaunchConfiguration('camera_name'),
            'svo_file':       LaunchConfiguration('svo_file'),
            'stream':         LaunchConfiguration('stream'),
            'node_name':      LaunchConfiguration('node_name'),
            'camera_model':   LaunchConfiguration('camera_model'),
            'base_frame':     LaunchConfiguration('base_frame'),
            'publish_urdf':   LaunchConfiguration('publish_urdf'),
            'cam_pos_x':      LaunchConfiguration('cam_pos_x'),
            'cam_pos_y':      LaunchConfiguration('cam_pos_y'),
            'cam_pos_z':      LaunchConfiguration('cam_pos_z'),
            'cam_roll':       LaunchConfiguration('cam_roll'),
            'cam_pitch':      LaunchConfiguration('cam_pitch'),
            'cam_yaw':        LaunchConfiguration('cam_yaw'),
        }.items()
    )

    # Group under the camera_name namespace
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('camera_name')),
        zed_launch
    ])

    return LaunchDescription(args + [group])
