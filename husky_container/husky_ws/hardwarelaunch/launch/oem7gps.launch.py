#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('novatel_oem7_driver')
    std_config_xml = os.path.join(pkg_share, 'config', 'std_driver_config.xml')

    # 1) Declare the same launch arguments
    oem7_if_arg = DeclareLaunchArgument(
        'oem7_if', default_value='Oem7ReceiverPort',
        description='Interface name for the Oem7 receiver'
    )
    oem7_tty_arg = DeclareLaunchArgument(
        'oem7_tty_name', default_value='/dev/ttyUSB2',
        description='TTY device for the Oem7 receiver'
    )
    oem7_baud_arg = DeclareLaunchArgument(
        'oem7_tty_baud', default_value='115200',
        description='Baud rate for the Oem7 receiver'
    )
    oem7_log_arg = DeclareLaunchArgument(
        'oem7_receiver_log', default_value='',
        description='Optional log filename for the receiver'
    )
    oem7_bist_arg = DeclareLaunchArgument(
        'oem7_bist', default_value='false',
        description='Run built-in self test'
    )

    # 2) Include the standard XML config (it still works as long as you keep it in config/)
    std_driver_config = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(std_config_xml),
        launch_arguments={'oem7_bist': LaunchConfiguration('oem7_bist')}.items()
    )

    # 3) Launch the actual driver node, passing your params exactly as before
    novatel_node = Node(
        package='novatel_oem7_driver',
        executable='novatel_gps_nodelet',  # or the proper executable name in ROS2 port
        name='novatel',
        parameters=[{
            '/novatel/oem7/receivers/main/oem7_if': LaunchConfiguration('oem7_if'),
            '/novatel/oem7/receivers/main/oem7_tty_name': LaunchConfiguration('oem7_tty_name'),
            '/novatel/oem7/receivers/main/oem7_tty_baud': LaunchConfiguration('oem7_tty_baud'),
            '/novatel/oem7/receivers/main/oem7_receiver_log_file': LaunchConfiguration('oem7_receiver_log'),
        }],
        output='screen'
    )

    return LaunchDescription([
        oem7_if_arg,
        oem7_tty_arg,
        oem7_baud_arg,
        oem7_log_arg,
        oem7_bist_arg,
        std_driver_config,
        novatel_node,
    ])
