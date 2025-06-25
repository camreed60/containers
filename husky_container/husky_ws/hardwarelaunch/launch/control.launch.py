#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Locate package shares
    hw_launch = get_package_share_directory('hardwarelaunch')
    hc_share  = get_package_share_directory('husky_control')

    # 1) Declare arguments (with optenv-style defaults)
    robot_ns_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=EnvironmentVariable('ROBOT_NAMESPACE', default_value='robot'),
        description='Top-level namespace for all Husky nodes'
    )
    enable_ekf_arg = DeclareLaunchArgument(
        'enable_ekf',
        default_value=EnvironmentVariable('ENABLE_EKF', default_value='false'),
        description='Whether to start the EKF localization'
    )
    config_extras_arg = DeclareLaunchArgument(
        'config_extras',
        default_value=EnvironmentVariable(
            'HUSKY_CONFIG_EXTRAS',
            default_value=os.path.join(hc_share, 'config', 'empty.yaml')
        ),
        description='Optional extra parameter file to load at the end'
    )
    laser_enabled_arg = DeclareLaunchArgument(
        'laser_enabled',
        default_value=EnvironmentVariable('HUSKY_LMS1XX_ENABLED', default_value='false'),
        description='Enable the LMS1XX laser'
    )
    realsense_enabled_arg = DeclareLaunchArgument(
        'realsense_enabled',
        default_value=EnvironmentVariable('HUSKY_REALSENSE_ENABLED', default_value='false'),
        description='Enable the Realsense camera'
    )
    urdf_extras_arg = DeclareLaunchArgument(
        'urdf_extras',
        default_value=EnvironmentVariable('HUSKY_URDF_EXTRAS', default_value=''),
        description='Extra URDF snippets (if any)'
    )

    # 2) Load robot description (forward args into description.launch.py)
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_launch, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'robot_namespace':    LaunchConfiguration('robot_namespace'),
            'laser_enabled':      LaunchConfiguration('laser_enabled'),
            'realsense_enabled':  LaunchConfiguration('realsense_enabled'),
            'urdf_extras':        LaunchConfiguration('urdf_extras'),
        }.items()
    )

    # 3) Load the main controller parameters
    control_params = PathJoinSubstitution([hc_share, 'config', 'control.yaml'])

    # 4) Spawn the controllers via the ROS 2 spawner
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='base_controller_spawner',
        output='screen',
        arguments=['husky_joint_publisher', 'husky_velocity_controller'],
        parameters=[control_params],
    )

    # 5) EKF localization (conditional)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        parameters=[PathJoinSubstitution([hc_share, 'config', 'localization.yaml'])],
        condition=IfCondition(LaunchConfiguration('enable_ekf')),
    )

    # 6) Interactive marker twist server
    marker_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_marker_server',
        output='screen'
    )

    # 7) Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen'
    )

    # 8) Twist mux
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[PathJoinSubstitution([hc_share, 'config', 'twist_mux.yaml'])],
        remappings=[('cmd_vel_out', 'husky_velocity_controller/cmd_vel')]
    )

    # 9) Optional extra config (ros1 used <rosparam load>; in ROS 2 you must pass this
    #    file into each node’s parameter list or load via CLI after launch. Here we
    #    declare it so you can wire it into your own nodes as needed.)
    #    You could, for example, append LaunchConfiguration('config_extras') to
    #    any Node(parameters=[...]) above to overlay those params.
    
    return LaunchDescription([
        robot_ns_arg,
        enable_ekf_arg,
        config_extras_arg,
        laser_enabled_arg,
        realsense_enabled_arg,
        urdf_extras_arg,
        description,
        spawner,
        ekf,
        marker_server,
        rsp,
        twist_mux,
    ])
