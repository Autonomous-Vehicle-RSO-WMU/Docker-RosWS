#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fusion_config = str(
        Path(get_package_share_directory('fusion_node')) / 'config' / 'fusion_node.yaml'
    )

    return LaunchDescription([
        Node(
            package='fusion_node',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[fusion_config],
        ),
    ])
