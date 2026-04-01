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
LAUNCH_LIDAR_BRINGUP      = True   # Ouster sensor driver
LAUNCH_CAMERA_BRINGUP     = True   # ZED camera driver // front camera
LAUNCH_LIDAR_CLUSTERING   = True   # LiDAR perception / clustering node
LAUNCH_CAMERA_DETECTION   = True    # Camera object detection node // front
USE_ZED_OD                = False  # True = ZED SDK OD adapter (TensorRT); False = ultralytics YOLO node
LAUNCH_FUSION             = True    # Sensor fusion node
LAUNCH_REAR_CAMERA_BRINGUP = True  # rear
LAUNCH_REAR_CAMERA_DETECTION = True  #rear



# --- LiDAR (Ouster) settings ---
LIDAR_SENSOR_IP           = '192.168.1.10'   # IP of the Ouster sensor
LIDAR_HOST_IP             = '192.168.1.69'    # IP of this machine on the LiDAR NIC
LIDAR_AZIMUTH_START       = '000000'          # Azimuth window start (millidegrees)
LIDAR_AZIMUTH_END         = '360000'          # Azimuth window end   (millidegrees)

# --- ZED camera settings ---
ZED_CAMERA_MODEL          = 'zed2i'          # zed | zedm | zed2 | zed2i | zedx | zedxm | zedxonegs | zedxone4k
ZED_CAMERA_NAME           = 'zed'             # Used as ROS namespace
ZED_SERIAL_NUMBER         = '37643796'         # ZED 2i front camera S/N
ZED__REAR_CAMERA_MODEL    = 'zed'             # zed | zedm | zed2 | zed2i | zedx | zedxm | zedxonegs | zedxone4k
ZED__REAR_CAMERA_NAME     = 'zed_rear'        # Used as ROS namespace (must be unique per camera)
ZED_REAR_SERIAL_NUMBER    = '23810'            # ZED original rear camera S/N

# --- Visualisation ---
LAUNCH_RVIZ               = True             # Launch RViz2

# --- Misc ---
USE_SIM_TIME              = False             # Set True when replaying bags / using a simulator

# ==============================================================================
#  END OF CONFIGURATION
# ==============================================================================


def generate_launch_description():
    actions = []

    # ── Print active subsystems ──────────────────────────────────────────────
    actions.append(LogInfo(msg='--- Sensor Fusion Launch ---'))
    actions.append(LogInfo(msg=f'  LiDAR bringup    : {LAUNCH_LIDAR_BRINGUP}'))
    actions.append(LogInfo(msg=f'  Camera bringup   : {LAUNCH_CAMERA_BRINGUP}'))
    actions.append(LogInfo(msg=f'  LiDAR clustering : {LAUNCH_LIDAR_CLUSTERING}'))
    actions.append(LogInfo(msg=f'  Camera detection : {LAUNCH_CAMERA_DETECTION} (ZED OD={USE_ZED_OD})'))
    actions.append(LogInfo(msg=f'  Fusion           : {LAUNCH_FUSION}'))
    actions.append(LogInfo(msg=f'  RViz2            : {LAUNCH_RVIZ}'))

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

    # ── 2. Camera Bringup (ZED driver) ──────────────────────────────────────
    if LAUNCH_CAMERA_BRINGUP:
        zed_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory('zed_wrapper'))
                    / 'launch'
                    / 'zed_camera.launch.py'
                )
            ),
            launch_arguments={
                'camera_model':  ZED_CAMERA_MODEL,
                'camera_name':   ZED_CAMERA_NAME,
                'serial_number': ZED_SERIAL_NUMBER,
                'publish_tf':    'false',    # Disabled: we publish our own static base_link → camera TF
                'publish_map_tf': 'false',   # No odom/map TF needed for static mount
            }.items(),
        )
        actions.append(zed_launch)

    if LAUNCH_REAR_CAMERA_BRINGUP:
        zed_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory('zed_wrapper'))
                    / 'launch'
                    / 'zed_camera.launch.py'
                )
            ),
            launch_arguments={
                'camera_model':  ZED__REAR_CAMERA_MODEL,
                'camera_name':   ZED__REAR_CAMERA_NAME,
                'serial_number': ZED_REAR_SERIAL_NUMBER,
                'publish_tf':    'false',    # Disabled: we publish our own static base_link → camera TF
                'publish_map_tf': 'false',   # No odom/map TF needed for static mount
            }.items(),
        )
        actions.append(zed_launch)

    # ── Static TF publishers ──────────────────────────────────────────────
    # Each sensor gets its own transform from base_link (at ground level).
    # LiDAR: 18.5 in (0.470 m) above ground, mounted backwards (yaw = π)
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.470',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_sensor',
        ],
    )
    actions.append(base_to_lidar_tf)

    # Front ZED: 4 ft (1.219 m) above ground, tilted 36° forward (pitch = +0.6283 rad)
    base_to_front_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_front_cam_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '1.219',
            '--roll', '0.0', '--pitch', '0.6283', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link',
        ],
    )
    actions.append(base_to_front_cam_tf)

    # Rear ZED 2i: 4 ft 3 in (1.295 m) above ground, 8.5 in (0.216 m) behind front,
    # facing backward (yaw = π), tilted 34° down (pitch = +0.5934 rad)
    base_to_rear_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rear_cam_tf',
        arguments=[
            '--x', '-0.216', '--y', '0.0', '--z', '1.295',
            '--roll', '0.0', '--pitch', '0.5934', '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_rear_camera_link',
        ],
    )
    actions.append(base_to_rear_cam_tf)

    # ── 3. LiDAR Clustering / Perception (reads /ouster/points) ──────
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

    # ── 4. Camera Detection ──────────────────────────────────────────────────
    if LAUNCH_CAMERA_DETECTION:
        if USE_ZED_OD:
            # ZED SDK OD adapter: converts ObjectsStamped → Detection2DArray
            adapter_config = str(
                Path(get_package_share_directory('camera_perception'))
                / 'config'
                / 'zed_od_adapter.yaml'
            )
            camera_node = Node(
                package='camera_perception',
                executable='zed_od_adapter_node',
                name='zed_od_adapter_node',
                output='screen',
                parameters=[adapter_config],
            )
        else:
            # Standalone YOLO node (requires ultralytics)
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
        if USE_ZED_OD:
            # ZED SDK OD adapter: converts ObjectsStamped → Detection2DArray
            rear_adapter_config = str(
                Path(get_package_share_directory('camera_perception'))
                / 'config'
                / 'zed_od_adapter_rear.yaml'
            )
            rear_camera_node = Node(
                package='camera_perception',
                executable='zed_od_adapter_node',
                name='zed_od_adapter_node_rear',
                output='screen',
                parameters=[rear_adapter_config],
            )
        else:
            # Standalone YOLO node — rear camera
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
        fusion_node = Node(
            package='fusion_node',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[fusion_config],
        )
        actions.append(fusion_node)

    # ── 6. RViz2 ─────────────────────────────────────────────────────────────
    if LAUNCH_RVIZ:
        # Pick the right RViz config based on which subsystems are active
        if LAUNCH_LIDAR_BRINGUP or LAUNCH_LIDAR_CLUSTERING:
            rviz_config = str(
                Path(get_package_share_directory('lidar_perception'))
                / 'config'
                / 'rviz_lidar.rviz'
            )
        else:
            # Camera-only: use ZED rviz config from launch file directory
            rviz_config = str(
                Path(__file__).parent / 'zed_rviz.rviz'
            )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )
        actions.append(rviz_node)

    return LaunchDescription(actions)
