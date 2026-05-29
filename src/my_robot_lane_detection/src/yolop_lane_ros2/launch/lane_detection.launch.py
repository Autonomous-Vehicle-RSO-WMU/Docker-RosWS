import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Find Paths ---
    package_share_dir = get_package_share_directory('yolop_lane_ros2')
    config_file_path = os.path.join(package_share_dir, 'config', 'lane_detection.yaml')

    # --- 2. Define Nodes ---
    lane_detection = Node(
        package='yolop_lane_ros2',
        executable='yolop_lane_node',
        name='yolop_lane_node',
        output='screen',
        parameters=[config_file_path]
    )

    # --- 3. Return Final Launch Description ---
    return LaunchDescription([
        lane_detection
    ])
