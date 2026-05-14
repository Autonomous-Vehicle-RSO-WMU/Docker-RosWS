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
    my_robot_description_share_dir = get_package_share_directory('my_robot_description')

    ros2_control_params = os.path.join(my_robot_description_share_dir, 'config', 'controllers.yaml')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution(
                [my_robot_description_share_dir, 'xacro', 'robot.urdf.xacro']
            ),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # --- 2. Define Launch Arguments

    # --- 3. Include Core Launch Files ---
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            my_robot_description_share_dir, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # --- 4. Define Nodes
    #control_node = Node(
    #    package='controller_manager',
    #    executable='ros2_control_node',
    #    parameters=[robot_description,
    #                ros2_control_params]
    #)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_control_params],
        remappings=[('~/robot_description', '/robot_description')]
    )
    
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    )

    # imu_broad_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['imu_broad', '--controller-manager', '/controller_manager']
    # )

    delay_diff_drive_spawner_after_joint_broad_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = joint_broad_spawner,
            on_exit = [diff_drive_spawner]
        )
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[('/cmd_vel_in', '/cmd_vel'), 
                    ('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

    # --- 5. Return Final Launch Description ---
    return LaunchDescription([
        control_node,
        rsp_launch,
        joint_broad_spawner,
        # imu_broad_spawner,
        delay_diff_drive_spawner_after_joint_broad_spawner,
        twist_stamper,
    ])