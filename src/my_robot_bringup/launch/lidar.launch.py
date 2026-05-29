#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_bringup    = LaunchConfiguration('launch_bringup')
    launch_clustering = LaunchConfiguration('launch_clustering')

    return LaunchDescription([
        DeclareLaunchArgument('launch_bringup',    default_value='true'),
        DeclareLaunchArgument('launch_clustering', default_value='true'),
        DeclareLaunchArgument('sensor_hostname',   default_value='192.168.1.58'),
        DeclareLaunchArgument('udp_dest',          default_value='192.168.1.10'),
        DeclareLaunchArgument(
            'timestamp_mode',
            default_value='TIME_FROM_ROS_TIME',
            description='Ouster timestamp mode. ROS time keeps message stamps aligned with TF clock.'
        ),
        DeclareLaunchArgument('azimuth_start',     default_value='315000'),
        DeclareLaunchArgument('azimuth_end',       default_value='45000'),

        # Ouster driver
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                str(Path(get_package_share_directory('ouster_ros')) / 'launch' / 'sensor.launch.xml')
            ),
            launch_arguments={
                'sensor_hostname':        LaunchConfiguration('sensor_hostname'),
                'udp_dest':               LaunchConfiguration('udp_dest'),
                'timestamp_mode':         LaunchConfiguration('timestamp_mode'),
                'viz':                    'false',
                'use_system_default_qos': 'true',
                'azimuth_window_start':   LaunchConfiguration('azimuth_start'),
                'azimuth_window_end':     LaunchConfiguration('azimuth_end'),
            }.items(),
            condition=IfCondition(launch_bringup),
        ),

        Node(
            package='topic_tools',
            executable='throttle',
            arguments=['messages', '/ouster/points', '1.0', '/ouster/points_throttled'],
        ),

        # LiDAR clustering / perception
        Node(
            package='lidar_perception',
            executable='lidar_perception_node',
            name='lidar_perception_node',
            output='screen',
            parameters=[
                str(Path(get_package_share_directory('lidar_perception')) / 'config' / 'lidar_perception.yaml')
            ],
            condition=IfCondition(launch_clustering),
        ),
    ])
