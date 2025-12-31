## FIX: must adjust old launch description (paths have changed)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- 1. Find Paths ---
    av_sim_core_share_dir = get_package_share_directory('av_sim_core')    # Find path to the packages (av_sim_core) share directory
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')  # Find path to the packages (nav2_bringup) share directory
    
    nav2_params_file = os.path.join(
        av_sim_core_share_dir, 'config', 'nav2_params.yaml')  # Constructs full path to params file from the av_sim_core share directory

    # --- 2. Define Launch Arguments ---
    use_map_server = LaunchConfiguration('use_map_server', default='False')  # Not using traditional static maps, so don't try and preload
    use_amcl = LaunchConfiguration('use_amcl', default='False')              # Not localizing on a static map

    # --- 3. Main NAV2 Bringup ---
    # Define the main NAV2 launch file (bringup_launch.py), with custom launch arguments
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_map_server': use_map_server,
            'use_amcl': use_amcl,
        }.items(),
    )

    # --- 4. Define and Launch Other Nodes ---
    # We DO NOT launch pure_pursuit.py or stop_controller.py, NAV2's controller_server replaces them.

    # --- Spawn Actors Node ---
    # Spawns all the other cars and pedestrians
    spawn_actors_node = Node(
        package='av_sim_core',
        executable='spawn_actors.py',
        name='spawn_actors',
        output='screen'
    )
    
    # --- Sensor Fusion Node ---
    # Fuses Lidar + Radar into the /carla/hero/fused_objects topic that the costmaps are subscribed to
    sensor_fusion_node = Node(
        package='av_sim_core',
        executable='sensor_fusion.py',
        name='sensor_fusion_node',
        output='screen'
    )

    # --- Vision Detector Node ---
    # Runs YOLO to detect stop signs
    vision_detector_node = Node(
        package='av_sim_core',
        executable='vision_detector.py',
        name='vision_detector',
        output='screen'
    )

    # --- 5. Create the Final Launch Description ---
    # Collect all individual nodes and launch actions to be returned for execution
    return LaunchDescription([
        nav2_bringup_launch,
        spawn_actors_node,
        sensor_fusion_node,
        vision_detector_node,
    ])