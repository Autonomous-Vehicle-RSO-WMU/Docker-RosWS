#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    lidar_pkg_share = get_package_share_directory('lidar_perception')
    rviz_config_default = str(Path(lidar_pkg_share) / 'config' / 'rviz_lidar.rviz')
    lidar_params = str(Path(lidar_pkg_share) / 'config' / 'lidar_perception.yaml')

    sensor_hostname = LaunchConfiguration('sensor_hostname')
    udp_dest = LaunchConfiguration('udp_dest')
    azimuth_window_start = LaunchConfiguration('azimuth_window_start')
    azimuth_window_end = LaunchConfiguration('azimuth_window_end')
    rviz_config = LaunchConfiguration('rviz_config')

    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value='192.168.1.69',
        description='Ouster sensor hostname or IP.'
    )
    udp_dest_arg = DeclareLaunchArgument(
        'udp_dest',
        default_value='192.168.1.134',
        description='Destination IP for UDP packets (this PC on the LiDAR NIC).'
    )
    azimuth_window_start_arg = DeclareLaunchArgument(
        'azimuth_window_start',
        default_value='135000',
        description='Azimuth window start in millidegrees.'
    )
    azimuth_window_end_arg = DeclareLaunchArgument(
        'azimuth_window_end',
        default_value='225000',
        description='Azimuth window end in millidegrees.'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_default,
        description='RViz config file.'
    )

    ouster_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            str(Path(get_package_share_directory('ouster_ros')) / 'launch' / 'sensor.launch.xml')
        ),
        launch_arguments={
            'sensor_hostname': sensor_hostname,
            'udp_dest': udp_dest,
            'viz': 'false',
            'use_system_default_qos': 'true',
            'azimuth_window_start': azimuth_window_start,
            'azimuth_window_end': azimuth_window_end,
        }.items()
    )

    rviz = ExecuteProcess(
        cmd=[
            '/usr/bin/env',
            '-i',
            [TextSubstitution(text='HOME='), EnvironmentVariable('HOME')],
            [TextSubstitution(text='DISPLAY='), LaunchConfiguration('display')],
            [TextSubstitution(text='XAUTHORITY='), LaunchConfiguration('xauthority')],
            [TextSubstitution(text='XDG_RUNTIME_DIR='), LaunchConfiguration('xdg_runtime_dir')],
            'PATH=/usr/bin:/bin:/opt/ros/humble/bin',
            'AMENT_PREFIX_PATH=/opt/ros/humble',
            'ROS_DISTRO=humble',
            'RVIZ_PLUGIN_PATH=/opt/ros/humble/lib',
            'LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu',
            '/opt/ros/humble/bin/rviz2',
            '-d',
            rviz_config,
        ],
        output='screen',
    )

    lidar_node = Node(
        package='lidar_perception',
        executable='lidar_perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            lidar_params,
        ],
    )

    display_arg = DeclareLaunchArgument(
        'display',
        default_value=EnvironmentVariable('DISPLAY'),
        description='X11 display for RViz.'
    )
    xauthority_arg = DeclareLaunchArgument(
        'xauthority',
        default_value=EnvironmentVariable('XAUTHORITY'),
        description='X11 authority file for RViz.'
    )
    xdg_runtime_dir_arg = DeclareLaunchArgument(
        'xdg_runtime_dir',
        default_value=TextSubstitution(text='/tmp/runtime-rviz'),
        description='Runtime dir for Qt.'
    )

    return LaunchDescription([
        sensor_hostname_arg,
        udp_dest_arg,
        azimuth_window_start_arg,
        azimuth_window_end_arg,
        rviz_config_arg,
        display_arg,
        xauthority_arg,
        xdg_runtime_dir_arg,
        ouster_launch,
        lidar_node,
        rviz,
    ])
