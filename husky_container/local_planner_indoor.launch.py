#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Declare all of your args exactly as in the ROS1 XML:
    sensor_offset_x = DeclareLaunchArgument('sensorOffsetX', default_value='0.24')
    sensor_offset_y = DeclareLaunchArgument('sensorOffsetY', default_value='0')
    sensor_offset_z = DeclareLaunchArgument('sensorOffsetZ', default_value='0.30')
    camera_offset_z = DeclareLaunchArgument('cameraOffsetZ', default_value='0')
    two_way_drive   = DeclareLaunchArgument('twoWayDrive',   default_value='false')
    max_speed       = DeclareLaunchArgument('maxSpeed',       default_value='0.60')
    autonomy_mode   = DeclareLaunchArgument('autonomyMode',   default_value='true')
    autonomy_speed  = DeclareLaunchArgument('autonomySpeed',  default_value='0.60')
    joy_to_speed    = DeclareLaunchArgument('joyToSpeedDelay',default_value='2.0')
    goal_x          = DeclareLaunchArgument('goalX',          default_value='0')
    goal_y          = DeclareLaunchArgument('goalY',          default_value='0')

    # helper to compute the pathFolder location
    pkg_local = get_package_share_directory('local_planner')
    path_folder = os.path.join(pkg_local, 'paths')

    # Node: localPlanner
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder':          path_folder,
            'vehicleLength':       1.2,
            'vehicleWidth':        0.9,
            'sensorOffsetX':       LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY':       LaunchConfiguration('sensorOffsetY'),
            'sensorOffsetZ':       LaunchConfiguration('sensorOffsetZ'),
            'twoWayDrive':         LaunchConfiguration('twoWayDrive'),
            'laserVoxelSize':      0.05,
            'terrainVoxelSize':    0.2,
            'useTerrainAnalysis':  True,
            'checkObstacle':       True,
            'checkRotObstacle':    True,
            'adjacentRange':       5.0,
            'obstacleHeightThre':  0.135,
            'groundHeightThre':    0.1,
            'costHeightThre':      0.1,
            'costScore':           8.0,
            'useCost':             False,
            'pointPerPathThre':    2,
            'minRelZ':             -1.0,
            'maxRelZ':             1.5,
            'maxSpeed':            LaunchConfiguration('maxSpeed'),
            'dirWeight':           0.05,
            'dirThre':             180.0,
            'dirToVehicle':        True,
            'pathScale':           2.25,
            'minPathScale':        0.75,
            'pathScaleStep':       0.25,
            'pathScaleBySpeed':    True,
            'minPathRange':        1.0,
            'pathRangeStep':       0.5,
            'pathRangeBySpeed':    True,
            'pathCropByGoal':      True,
            'autonomyMode':        LaunchConfiguration('autonomyMode'),
            'autonomySpeed':       LaunchConfiguration('autonomySpeed'),
            'joyToSpeedDelay':     LaunchConfiguration('joyToSpeedDelay'),
            'joyToCheckObstacleDelay': 5.0,
            'goalClearRange':      0.65,
            'goalX':               LaunchConfiguration('goalX'),
            'goalY':               LaunchConfiguration('goalY'),
        }]
    )

    # Node: pathFollower
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX':    LaunchConfiguration('sensorOffsetX'),
            'sensorOffsetY':    LaunchConfiguration('sensorOffsetY'),
            'pubSkipNum':       1,
            'twoWayDrive':      LaunchConfiguration('twoWayDrive'),
            'lookAheadDis':     4.0,
            'yawRateGain':      0.30,
            'stopYawRateGain':  3.0,
            'maxYawRate':       30.0,
            'maxSpeed':         LaunchConfiguration('maxSpeed'),
            'maxAccel':         0.5,
            'switchTimeThre':   2.0,
            'dirDiffThre':      0.2,
            'stopDisThre':      0.8,
            'slowDwnDisThre':   1.5,
            'useInclRateToSlow':    False,
            'inclRateThre':         50.0,
            'slowRate1':            0.25,
            'slowRate2':            0.3,
            'slowTime1':            2.0,
            'slowTime2':            2.0,
            'useInclToStop':        False,
            'inclThre':             50.0,
            'stopTime':             5.0,
            'noRotAtStop':          False,
            'noRotAtGoal':          True,
            'autonomyMode':         LaunchConfiguration('autonomyMode'),
            'autonomySpeed':        LaunchConfiguration('autonomySpeed'),
            'joyToSpeedDelay':      LaunchConfiguration('joyToSpeedDelay'),
        }]
    )

    return LaunchDescription([
        # args
        sensor_offset_x, sensor_offset_y, sensor_offset_z,
        camera_offset_z, two_way_drive, max_speed,
        autonomy_mode, autonomy_speed, joy_to_speed,
        goal_x, goal_y,
        # nodes
        local_planner_node,
        path_follower_node,
    ])
