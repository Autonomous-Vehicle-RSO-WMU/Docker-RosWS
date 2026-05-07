#!/usr/bin/env python3
"""
Sensor Fusion Modular Launch File
==================================
Edit the GLOBAL CONFIGURATION section below to enable/disable subsystems
and configure sensor parameters before launching.
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
)
from launch_ros.actions import Node


# ==============================================================================
#  GLOBAL CONFIGURATION  —  Edit these before launching
# ==============================================================================

# --- Subsystem enable/disable flags ---
LAUNCH_LIDAR_BRINGUP         = False  # Ouster sensor driver
LAUNCH_CAMERA_BRINGUP        = True   # ZED front camera driver
LAUNCH_REAR_CAMERA_BRINGUP   = True   # ZED rear camera driver
LAUNCH_LIDAR_CLUSTERING      = False  # LiDAR perception / clustering node
LAUNCH_CAMERA_DETECTION      = True   # Front camera YOLO detection
LAUNCH_REAR_CAMERA_DETECTION = True   # Rear camera YOLO detection
LAUNCH_FUSION                = False  # LiDAR-camera fusion node
LAUNCH_RVIZ                  = True   # Launch RViz2

# --- LiDAR (Ouster) settings ---
LIDAR_SENSOR_IP    = '192.168.1.10'  # IP of the Ouster sensor
LIDAR_HOST_IP      = '192.168.1.69'  # IP of this machine on the LiDAR NIC
LIDAR_AZIMUTH_START = '000000'       # Azimuth window start (millidegrees)
LIDAR_AZIMUTH_END   = '360000'       # Azimuth window end   (millidegrees)

# --- ZED camera settings ---
ZED_CAMERA_MODEL       = 'zed2i'     # Front camera model
ZED_CAMERA_NAME        = 'zed'       # ROS namespace for front camera
ZED_SERIAL_NUMBER      = '37643796'  # ZED 2i front S/N

ZED_REAR_CAMERA_MODEL  = 'zed'       # Rear camera model
ZED_REAR_CAMERA_NAME   = 'zed_rear'  # ROS namespace for rear camera (must be unique)
ZED_REAR_SERIAL_NUMBER = '23810'     # ZED original rear S/N

# --- Misc ---
USE_SIM_TIME = False  # Set True when replaying bags / using a simulator

# ==============================================================================
#  END OF CONFIGURATION
# ==============================================================================


def generate_launch_description():
    actions = []

    bringup_share = get_package_share_directory('my_robot_bringup')

    # ── Print active subsystems ──────────────────────────────────────────────
    actions.append(LogInfo(msg='--- Sensor Fusion Launch ---'))
    actions.append(LogInfo(msg=f'  LiDAR bringup         : {LAUNCH_LIDAR_BRINGUP}'))
    actions.append(LogInfo(msg=f'  Camera bringup        : {LAUNCH_CAMERA_BRINGUP}'))
    actions.append(LogInfo(msg=f'  Rear camera bringup   : {LAUNCH_REAR_CAMERA_BRINGUP}'))
    actions.append(LogInfo(msg=f'  LiDAR clustering      : {LAUNCH_LIDAR_CLUSTERING}'))
    actions.append(LogInfo(msg=f'  Camera detection      : {LAUNCH_CAMERA_DETECTION}'))
    actions.append(LogInfo(msg=f'  Rear camera detection : {LAUNCH_REAR_CAMERA_DETECTION}'))
    actions.append(LogInfo(msg=f'  Fusion                : {LAUNCH_FUSION}'))
    actions.append(LogInfo(msg=f'  RViz2                 : {LAUNCH_RVIZ}'))

    # ── 1. LiDAR Bringup (Ouster driver) ────────────────────────────────────
    if LAUNCH_LIDAR_BRINGUP:
        ouster_launch = IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory('ouster_ros'))
                    / 'launch'
                    / 'sensor.launch.xml'
                )
            ),
            launch_arguments={
                'sensor_hostname':        LIDAR_SENSOR_IP,
                'udp_dest':               LIDAR_HOST_IP,
                'viz':                    'false',
                'use_system_default_qos': 'true',
                'azimuth_window_start':   LIDAR_AZIMUTH_START,
                'azimuth_window_end':     LIDAR_AZIMUTH_END,
            }.items(),
        )
        actions.append(ouster_launch)

    # ── 2. Camera Bringup (ZED drivers) ─────────────────────────────────────
    # TF (base_link → camera_link / camera_link_rear / os_sensor) is provided
    # by robot_state_publisher via the xacro URDF — not by the ZED driver.
    if LAUNCH_CAMERA_BRINGUP:
        front_zed_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory('zed_wrapper'))
                    / 'launch'
                    / 'zed_camera.launch.py'
                )
            ),
            launch_arguments={
                'camera_model':        ZED_CAMERA_MODEL,
                'camera_name':         ZED_CAMERA_NAME,
                'serial_number':       ZED_SERIAL_NUMBER,
                'publish_tf':          'false',
                'publish_map_tf':      'false'#,
              #  'param_overrides':     'debug.disable_nitros:=true',
            }.items(),
        )
        actions.append(front_zed_launch)

    if LAUNCH_REAR_CAMERA_BRINGUP:
        rear_zed_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory('zed_wrapper'))
                    / 'launch'
                    / 'zed_camera.launch.py'
                )
            ),
            launch_arguments={
                'camera_model':        ZED_REAR_CAMERA_MODEL,
                'camera_name':         ZED_REAR_CAMERA_NAME,
                'serial_number':       ZED_REAR_SERIAL_NUMBER,
                'publish_tf':          'false',
                'publish_map_tf':      'false',
                'param_overrides':     'debug.disable_nitros:=true',
            }.items(),
        )
        actions.append(rear_zed_launch)

    # ── 3. LiDAR Perception (reads /ouster/points) ──────────────────────────
    if LAUNCH_LIDAR_CLUSTERING:
        lidar_config = str(
            Path(get_package_share_directory('lidar_perception'))
            / 'config'
            / 'lidar_perception.yaml'
        )
        lidar_node = Node(
            package='lidar_perception',
            executable='lidar_perception_node',
            name='lidar_perception_node',
            output='screen',
            parameters=[lidar_config],
        )
        actions.append(lidar_node)

    # ── 4. Camera Detection (YOLO) ───────────────────────────────────────────
    if LAUNCH_CAMERA_DETECTION:
        camera_config = str(
            Path(get_package_share_directory('camera_perception'))
            / 'config'
            / 'camera_perception.yaml'
        )
        camera_node = Node(
            package='camera_perception',
            executable='camera_perception_node',
            name='camera_perception_node',
            output='screen',
            parameters=[camera_config],
        )
        actions.append(camera_node)

    if LAUNCH_REAR_CAMERA_DETECTION:
        rear_camera_config = str(
            Path(get_package_share_directory('camera_perception'))
            / 'config'
            / 'camera_perception_rear.yaml'
        )
        rear_camera_node = Node(
            package='camera_perception',
            executable='camera_perception_node',
            name='camera_perception_node_rear',
            output='screen',
            parameters=[rear_camera_config],
        )
        actions.append(rear_camera_node)

    # ── 5. Sensor Fusion ─────────────────────────────────────────────────────
    if LAUNCH_FUSION:
        fusion_config = str(
            Path(get_package_share_directory('fusion_node'))
            / 'config'
            / 'fusion_node.yaml'
        )
        fuse_node = Node(
            package='fusion_node',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[fusion_config],
        )
        actions.append(fuse_node)

    # ── 6. RViz2 ─────────────────────────────────────────────────────────────
    # if LAUNCH_RVIZ:
    #     if LAUNCH_LIDAR_BRINGUP or LAUNCH_LIDAR_CLUSTERING:
    #         rviz_config = str(
    #             Path(get_package_share_directory('lidar_perception'))
    #             / 'config'
    #             / 'rviz_lidar.rviz'
    #         )
    #     else:
    #         rviz_config = str(
    #             Path(bringup_share) / 'launch' / 'zed_rviz.rviz'
    #         )
    #     rviz_node = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         arguments=['-d', rviz_config],
    #     )
    #     actions.append(rviz_node)

    return LaunchDescription(actions)