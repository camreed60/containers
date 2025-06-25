#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Locate package shares
    vd_driver_share = get_package_share_directory('velodyne_driver')
    vd_pc_share     = get_package_share_directory('velodyne_pointcloud')

    # 1) Declare all your launch arguments
    calibration_arg = DeclareLaunchArgument(
        'calibration',
        default_value=os.path.join(vd_pc_share, 'params', '32db.yaml'),
        description='Path to VLP-32C calibration file'
    )
    device_ip_arg = DeclareLaunchArgument(
        'device_ip', default_value='192.168.1.201',
        description='IP of the Velodyne device'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='lidar_frame',
        description='TF frame ID for the pointcloud'
    )
    manager_arg = DeclareLaunchArgument(
        'manager',
        # default to "<frame_id>_nodelet_manager"
        default_value=[LaunchConfiguration('frame_id'), TextSubstitution(text='_nodelet_manager')],
        description='Name of the nodelet manager'
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='1300.0',
        description='Maximum range in meters'
    )
    min_range_arg = DeclareLaunchArgument(
        'min_range', default_value='0.4',
        description='Minimum range in meters'
    )
    pcap_arg = DeclareLaunchArgument(
        'pcap', default_value='',
        description='Optional path to .pcap file'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='2368',
        description='UDP port to listen on'
    )
    read_fast_arg = DeclareLaunchArgument(
        'read_fast', default_value='false',
        description='Enable fast UDP read'
    )
    read_once_arg = DeclareLaunchArgument(
        'read_once', default_value='false',
        description='Read packets only once (for pcap)'
    )
    repeat_delay_arg = DeclareLaunchArgument(
        'repeat_delay', default_value='0.0',
        description='Delay between pcap repeats'
    )
    rpm_arg = DeclareLaunchArgument(
        'rpm', default_value='600.0',
        description='Rotation speed'
    )
    gps_time_arg = DeclareLaunchArgument(
        'gps_time', default_value='false',
        description='Use GPS timestamps'
    )
    pcap_time_arg = DeclareLaunchArgument(
        'pcap_time', default_value='false',
        description='Use pcap packet timestamps'
    )
    cut_angle_arg = DeclareLaunchArgument(
        'cut_angle', default_value='-0.01',
        description='Start/stop angle for cut'
    )
    ts_first_arg = DeclareLaunchArgument(
        'timestamp_first_packet', default_value='false',
        description='Timestamp from first packet'
    )
    ring_arg = DeclareLaunchArgument(
        'laserscan_ring', default_value='-1',
        description='Which ring to extract for LaserScan'
    )
    resolution_arg = DeclareLaunchArgument(
        'laserscan_resolution', default_value='0.007',
        description='Angular resolution for LaserScan'
    )
    organize_arg = DeclareLaunchArgument(
        'organize_cloud', default_value='false',
        description='Output organized PointCloud2'
    )

    # 2) Include the driver nodelet manager (port to Python .launch.py)
    driver_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vd_driver_share, 'launch', 'nodelet_manager.launch.py')
        ),
        launch_arguments={
            'device_ip': LaunchConfiguration('device_ip'),
            'frame_id':  LaunchConfiguration('frame_id'),
            'manager':   LaunchConfiguration('manager'),
            'model':     '32E',
            'pcap':      LaunchConfiguration('pcap'),
            'port':      LaunchConfiguration('port'),
            'read_fast': LaunchConfiguration('read_fast'),
            'read_once': LaunchConfiguration('read_once'),
            'repeat_delay': LaunchConfiguration('repeat_delay'),
            'rpm':          LaunchConfiguration('rpm'),
            'gps_time':     LaunchConfiguration('gps_time'),
            'pcap_time':    LaunchConfiguration('pcap_time'),
            'cut_angle':    LaunchConfiguration('cut_angle'),
            'timestamp_first_packet': LaunchConfiguration('timestamp_first_packet'),
        }.items()
    )

    # 3) Include the transform nodelet
    transform_nodelet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vd_pc_share, 'launch', 'transform_nodelet.launch.py')
        ),
        launch_arguments={
            'model':        '32E',
            'calibration':  LaunchConfiguration('calibration'),
            'manager':      LaunchConfiguration('manager'),
            'fixed_frame':  '',
            'target_frame': '',
            'max_range':    LaunchConfiguration('max_range'),
            'min_range':    LaunchConfiguration('min_range'),
            'organize_cloud': LaunchConfiguration('organize_cloud'),
        }.items()
    )

    # 4) Include the laserscan nodelet
    laserscan_nodelet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vd_pc_share, 'launch', 'laserscan_nodelet.launch.py')
        ),
        launch_arguments={
            'manager':    LaunchConfiguration('manager'),
            'ring':       LaunchConfiguration('laserscan_ring'),
            'resolution': LaunchConfiguration('laserscan_resolution'),
        }.items()
    )

    return LaunchDescription([
        # args
        calibration_arg,
        device_ip_arg,
        frame_id_arg,
        manager_arg,
        max_range_arg,
        min_range_arg,
        pcap_arg,
        port_arg,
        read_fast_arg,
        read_once_arg,
        repeat_delay_arg,
        rpm_arg,
        gps_time_arg,
        pcap_time_arg,
        cut_angle_arg,
        ts_first_arg,
        ring_arg,
        resolution_arg,
        organize_arg,

        # includes
        driver_manager,
        transform_nodelet,
        laserscan_nodelet,
    ])
