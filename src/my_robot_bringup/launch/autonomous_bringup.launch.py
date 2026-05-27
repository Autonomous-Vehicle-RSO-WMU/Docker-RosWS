import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- 1. Find Paths ---
    my_robot_localization_share_dir = get_package_share_directory('my_robot_localization')
    my_robot_navigation_share_dir = get_package_share_directory('my_robot_navigation')
    my_robot_follower_share_dir = get_package_share_directory('my_robot_igvc_mission')

    # --- 2. Define Launch Arguments

    # --- 3. Include Core Launch Files ---
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_localization_share_dir, 'launch', 'gps_localization_launch.py'
        )]),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_navigation_share_dir, 'launch', 'nav2.launch.py'
        )]),
        launch_arguments = {'use_sim_time': 'false'}.items()
    )

    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_follower_share_dir, 'launch', 'mission_launch.py'
        )]),
        launch_arguments = {'use_sim_time': 'false'}.items()
    ) 
    

    # --- 4. Define Nodes

    # --- 5. Return Final Launch Description ---
    return LaunchDescription([
        localization_launch,
        navigation_launch
    ])