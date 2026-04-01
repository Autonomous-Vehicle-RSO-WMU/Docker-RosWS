#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('fusion_node')
    config = os.path.join(pkg_share, 'config', 'fusion_node.yaml')

    return LaunchDescription([
        Node(
            package='fusion_node',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[config],
        ),
    ])
