#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    camera_pkg_share = get_package_share_directory('camera_perception')
    rviz_config_default = str(Path(camera_pkg_share) / 'config' / 'rviz_camera.rviz')
    camera_config = str(Path(camera_pkg_share) / 'config' / 'camera_perception.yaml')

    camera_model = LaunchConfiguration('camera_model')
    rviz_config = LaunchConfiguration('rviz_config')

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed',
        description='ZED camera model name used by zed_wrapper.'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_default,
        description='RViz config file.'
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('zed_wrapper')) / 'launch' / 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': camera_model,
            'publish_left_right': 'true',
        }.items()
    )

    camera_node = Node(
        package='camera_perception',
        executable='camera_perception_node',
        name='camera_perception_node',
        output='screen',
        parameters=[camera_config],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        camera_model_arg,
        rviz_config_arg,
        zed_launch,
        camera_node,
        rviz,
    ])
