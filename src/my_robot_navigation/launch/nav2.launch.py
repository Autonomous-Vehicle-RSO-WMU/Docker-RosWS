import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- 1. Find Paths ---
    my_robot_navigation_share_dir = get_package_share_directory('my_robot_navigation')    # Find path to the package's (my_robot_navigation) share directory
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')  # Find path to the package's (nav2_bringup) share directory
    
    nav2_params_file = os.path.join(
        my_robot_navigation_share_dir, 'config', 'nav2_params.yaml')  # Constructs full path to params file from the my_robot_navigation share directory

    # --- 2. Define Launch Arguments ---
    use_map_server = LaunchConfiguration('use_map_server', default='False')  # Not using traditional static maps, so don't try and preload
    use_amcl = LaunchConfiguration('use_amcl', default='False')              # Not localizing on a static map
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # --- 3. Launch NAV2 Bringup ---
    # Define the main NAV2 launch file (bringup_launch.py), with custom launch arguments
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            # 'use_map_server': use_map_server,
            # 'use_amcl': use_amcl,
            'use_sim_time': use_sim_time,
            # 'map': '',
        }.items(),
    )

    # --- 4. Define Nodes ---
    # We DO NOT launch pure_pursuit.py or stop_controller.py, NAV2's controller_server replaces them.
    
    # --- Sensor Fusion Node ---  #UPDATE when sensor_fusion.py working
    # Fuses Lidar + Radar into the /carla/hero/fused_objects topic that the costmaps are subscribed to
    # sensor_fusion_node = Node(
    #     package='my_robot_localization',
    #     executable='sensor_fusion.py',
    #     name='sensor_fusion_node',
    #     output='screen'
    # )

    # --- Vision Detector Node ---  #UPDATE when vision_detector.py working
    # Runs YOLO to detect stop signs
    # vision_detector_node = Node(
    #     package='my_robot_perception',
    #     executable='vision_detector.py',
    #     name='vision_detector',
    #     output='screen'
    # )

    # --- 5. Return Final Launch Description ---
    # Collect all individual nodes and launch actions to be returned for execution
    return LaunchDescription([
        nav2_bringup_launch,
        #sensor_fusion_node,
        #vision_detector_node,
    ])