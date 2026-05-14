#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zed_wrapper_launch = str(
        Path(get_package_share_directory('zed_wrapper')) / 'launch' / 'zed_camera.launch.py'
    )
    cam_pkg = get_package_share_directory('camera_perception')

    return LaunchDescription([
        DeclareLaunchArgument('launch_front_bringup',    default_value='true'),
        DeclareLaunchArgument('launch_rear_bringup',     default_value='true'),
        DeclareLaunchArgument('launch_front_detection',  default_value='true'),
        DeclareLaunchArgument('launch_rear_detection',   default_value='true'),

        DeclareLaunchArgument('front_camera_model',  default_value='zed2i'),
        DeclareLaunchArgument('front_camera_name',   default_value='zed'),
        DeclareLaunchArgument('front_serial_number', default_value='37643796'),

        DeclareLaunchArgument('rear_camera_model',   default_value='zed'),
        DeclareLaunchArgument('rear_camera_name',    default_value='zed_rear'),
        DeclareLaunchArgument('rear_serial_number',  default_value='23810'),

        # Front ZED driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_wrapper_launch),
            launch_arguments={
                'camera_model':   LaunchConfiguration('front_camera_model'),
                'camera_name':    LaunchConfiguration('front_camera_name'),
                'serial_number':  LaunchConfiguration('front_serial_number'),
                'publish_tf':     'false',
                'publish_map_tf': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_front_bringup')),
        ),

        # Rear ZED driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_wrapper_launch),
            launch_arguments={
                'camera_model':   LaunchConfiguration('rear_camera_model'),
                'camera_name':    LaunchConfiguration('rear_camera_name'),
                'serial_number':  LaunchConfiguration('rear_serial_number'),
                'publish_tf':     'false',
                'publish_map_tf': 'false',
                'param_overrides': 'debug.disable_nitros:=true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_rear_bringup')),
        ),

        # Front camera YOLO detection
        Node(
            package='camera_perception',
            executable='camera_perception_node',
            name='camera_perception_node',
            output='screen',
            parameters=[str(Path(cam_pkg) / 'config' / 'camera_perception.yaml')],
            condition=IfCondition(LaunchConfiguration('launch_front_detection')),
        ),

        # Rear camera YOLO detection
        Node(
            package='camera_perception',
            executable='camera_perception_node',
            name='camera_perception_node_rear',
            output='screen',
            parameters=[str(Path(cam_pkg) / 'config' / 'camera_perception_rear.yaml')],
            condition=IfCondition(LaunchConfiguration('launch_rear_detection')),
        ),
    ])
