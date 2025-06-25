#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declare launch arguments with your new defaults
    args = [
        DeclareLaunchArgument('sensorOffsetX', default_value='0.22'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetZ', default_value='0.30'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0'),
        DeclareLaunchArgument('twoWayDrive', default_value='false'),
        DeclareLaunchArgument('maxSpeed', default_value='10.0'),
        DeclareLaunchArgument('autonomyMode', default_value='true'),
        DeclareLaunchArgument('autonomySpeed', default_value='5.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),
        DeclareLaunchArgument('goalX', default_value='0'),
        DeclareLaunchArgument('goalY', default_value='0'),
    ]

    # 2) Path to the 'paths' folder in local_planner
    pkg_local = get_package_share_directory('local_planner')
    path_folder = os.path.join(pkg_local, 'paths')

    # 3) localPlanner node
    local_planner = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder':            path_folder,
            'vehicleLength':         1.4,
            'vehicleWidth':          1.40,
            'sensorOffsetX':         LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY':         LaunchConfiguration('sensorOffsetY'),
            'sensorOffsetZ':         LaunchConfiguration('sensorOffsetZ'),
            'twoWayDrive':           LaunchConfiguration('twoWayDrive'),
            'laserVoxelSize':        0.05,
            'terrainVoxelSize':      0.2,
            'useTerrainAnalysis':    True,
            'checkObstacle':         True,
            'checkRotObstacle':      True,
            'adjacentRange':         4.25,
            'obstacleHeightThre':    0.15,
            'groundHeightThre':      0.1,
            'costHeightThre':        0.1,
            'costScore':             0.8,
            'useCost':               True,
            'pointPerPathThre':      2,
            'minRelZ':               -1.0,
            'maxRelZ':               1.5,
            'maxSpeed':              LaunchConfiguration('maxSpeed'),
            'dirWeight':             0.02,
            'dirThre':               90.0,
            'dirToVehicle':          True,
            'pathScale':             1.25,
            'minPathScale':          0.75,
            'pathScaleStep':         0.25,
            'pathScaleBySpeed':      True,
            'minPathRange':          1.0,
            'pathRangeStep':         0.5,
            'pathRangeBySpeed':      True,
            'pathCropByGoal':        True,
            'autonomyMode':          LaunchConfiguration('autonomyMode'),
            'autonomySpeed':         LaunchConfiguration('autonomySpeed'),
            'joyToSpeedDelay':       LaunchConfiguration('joyToSpeedDelay'),
            'joyToCheckObstacleDelay': 5.0,
            'goalClearRange':        0.5,
            'goalX':                 LaunchConfiguration('goalX'),
            'goalY':                 LaunchConfiguration('goalY'),
        }]
    )

    # 4) pathFollower node
    path_follower = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX':       LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY':       LaunchConfiguration('sensorOffsetY'),
            'pubSkipNum':          1,
            'twoWayDrive':         LaunchConfiguration('twoWayDrive'),
            'lookAheadDis':        4.6,
            'yawRateGain':         40.0,
            'stopYawRateGain':     8.0,
            'maxYawRate':          25.0,
            'maxSpeed':            LaunchConfiguration('maxSpeed'),
            'maxAccel':            5.0,
            'switchTimeThre':      1.0,
            'dirDiffThre':         0.2,
            'stopDisThre':         0.4,
            'slowDwnDisThre':      1.20,
            'useInclRateToSlow':   False,
            'inclRateThre':        120.0,
            'slowRate1':           0.25,
            'slowRate2':           0.5,
            'slowTime1':           2.0,
            'slowTime2':           2.0,
            'useInclToStop':       False,
            'inclThre':            45.0,
            'stopTime':            5.0,
            'noRotAtStop':         True,
            'noRotAtGoal':         True,
            'autonomyMode':        LaunchConfiguration('autonomyMode'),
            'autonomySpeed':       LaunchConfiguration('autonomySpeed'),
            'joyToSpeedDelay':     LaunchConfiguration('joyToSpeedDelay'),
        }]
    )

    # 5) static transforms
    tf_vehicle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicleTransPublisher',
        arguments=[
            '-'+LaunchConfiguration('sensorOffsetX'),   # notice the leading '-'
            '-'+LaunchConfiguration('sensorOffsetY'),
            '0',
            '0', '0', '0',
            '/sensor', '/vehicle', '1000'
        ]
    )
    tf_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensorTransPublisher',
        arguments=[
            '0', '0', LaunchConfiguration('cameraOffsetZ'),
            '-1.5707963', '0', '-1.5707963',
            '/sensor', '/camera', '1000'
        ]
    )

    return LaunchDescription(args + [
        local_planner,
        path_follower,
        tf_vehicle,
        tf_sensor,
    ])
