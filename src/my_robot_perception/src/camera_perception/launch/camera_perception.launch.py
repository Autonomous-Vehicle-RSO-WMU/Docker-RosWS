#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_perception')
    config = os.path.join(pkg_share, 'config', 'camera_perception.yaml')

    return LaunchDescription([
        Node(
            package='camera_perception',
            executable='camera_perception_node',
            name='camera_perception_node',
            output='screen',
            parameters=[config],
        ),
    ])
