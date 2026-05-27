import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Find Paths ---
    package_share_dir = get_package_share_directory('my_robot_igvc_mission')
    config_file_path = os.path.join(package_share_dir, 'config', 'mission_config.yaml')

    # --- 2. Define Nodes ---
    waypoint_follower = Node(
        package='my_robot_igvc_mission',
        executable='waypoint_follower',
        name='waypoint_params',
        output='screen',
        parameters=[config_file_path]
    )

    # --- 3. Return Final Launch Description ---
    return LaunchDescription([
        waypoint_follower
    ])
