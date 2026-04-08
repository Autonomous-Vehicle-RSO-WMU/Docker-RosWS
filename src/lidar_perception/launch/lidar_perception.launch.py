#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_path = str(
        Path(get_package_share_directory('lidar_perception')) / 'config' / 'lidar_perception.yaml'
    )
    return LaunchDescription([
        Node(
            package='lidar_perception',
            executable='lidar_perception_node',
            name='lidar_perception_node',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                params_path,
            ],
        ),
    ])
