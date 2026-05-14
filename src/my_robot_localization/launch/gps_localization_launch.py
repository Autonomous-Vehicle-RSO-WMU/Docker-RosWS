import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	# Path declarations.
	package_share_dir = get_package_share_directory('my_robot_localization')
	config_file_path = os.path.join(package_share_dir, 'config', 'ekf_config.yaml')
	
	# Launch configurations.
	use_sim_time = LaunchConfiguration('use_sim_time')
	
	# Launch arguments.
	sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        )
	
	# Actions.
	# Start navsat transform node.
	navsat_transform = Node(
		package = 'robot_localization',
		executable = 'navsat_transform_node',
		name = 'navsat_transform',
		output = 'screen',
		parameters = [config_file_path, {'use_sim_time': use_sim_time}], 
		remappings = [
			('imu/data', 'imu/data'),
			('gps/fix', 'gps/fix'),
			('gps/filtered', 'gps/filtered'),
			('odometry/gps', 'odometry/gps'),
			('odometry/filtered', 'odometry/global')
		]
	)
	
	# Start robot localization using Extended Kalman filter map->odom transform
	robot_localization_global = Node(
		package = 'robot_localization',
		executable = 'ekf_node',
		name = 'ekf_filter_node_map',
		output = 'screen',
		parameters = [config_file_path, {'use_sim_time': use_sim_time}], 
		remappings = [
			('odometry/filtered', 'odometry/global'),
			('/set_pose', '/initialpose')
		]
	)
	
	# Start robot localization using Extended Kalman filter odom->base_footprint transform
	robot_localization_local = Node(
		package = 'robot_localization',
		executable = 'ekf_node',
		name = 'ekf_filter_node_odom',
		output = 'screen',
		parameters = [config_file_path, {'use_sim_time': use_sim_time}], 
		remappings = [
			('odometry/filtered', 'odometry/local'),
			('/set_pose', '/initialpose')
		]
	)
	
	return LaunchDescription([
		sim_time_arg,
		navsat_transform,
		robot_localization_global,
		robot_localization_local
	])
