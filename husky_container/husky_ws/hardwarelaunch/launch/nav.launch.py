#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declare launch arguments
    camera_offset_arg = DeclareLaunchArgument(
        'cameraOffsetZ', default_value='0',
        description='Z offset of the camera'
    )
    vehicle_x_arg = DeclareLaunchArgument(
        'vehicleX', default_value='0',
        description='X coordinate of the goal'
    )
    vehicle_y_arg = DeclareLaunchArgument(
        'vehicleY', default_value='0',
        description='Y coordinate of the goal'
    )
    check_terrain_arg = DeclareLaunchArgument(
        'checkTerrainConn', default_value='true',
        description='Whether to check terrain connectivity'
    )

    # 2) Include local_planner.launch.py with remapped args
    local_planner_share = get_package_share_directory('local_planner')
    local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_planner_share, 'launch', 'local_planner.launch.py')
        ),
        launch_arguments={
            'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
            'goalX':         LaunchConfiguration('vehicleX'),
            'goalY':         LaunchConfiguration('vehicleY'),
        }.items()
    )

    # 3) Include terrain_analysis.launch.py
    terrain_analysis_share = get_package_share_directory('terrain_analysis')
    terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_analysis_share, 'launch', 'terrain_analysis.launch.py')
        )
    )

    # 4) Include terrain_analysis_ext.launch.py with its arg
    terrain_ext_share = get_package_share_directory('terrain_analysis_ext')
    terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_ext_share, 'launch', 'terrain_analysis_ext.launch.py')
        ),
        launch_arguments={
            'checkTerrainConn': LaunchConfiguration('checkTerrainConn')
        }.items()
    )

    # 5) Include sensor_scan_generation.launch.py
    ssg_share = get_package_share_directory('sensor_scan_generation')
    sensor_scan_generation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ssg_share, 'launch', 'sensor_scan_generation.launch.py')
        )
    )

    # 6) Include loam_interface.launch.py
    loam_share = get_package_share_directory('loam_interface')
    loam_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loam_share, 'launch', 'loam_interface.launch.py')
        )
    )

    # 7) Launch RViz2 with a nice prefix
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizGA',
        respawn=True,
        prefix='nice',
        output='screen'
    )

    return LaunchDescription([
        camera_offset_arg,
        vehicle_x_arg,
        vehicle_y_arg,
        check_terrain_arg,
        local_planner,
        terrain_analysis,
        terrain_analysis_ext,
        sensor_scan_generation,
        loam_interface,
        rviz,
    ])
