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
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

# --- LiDAR (Ouster) settings ---
LIDAR_SENSOR_IP     = '192.168.1.10'  # IP of the Ouster sensor
LIDAR_HOST_IP       = '192.168.1.69'  # IP of this machine on the LiDAR NIC
LIDAR_AZIMUTH_START = '000000'        # Azimuth window start (millidegrees)
LIDAR_AZIMUTH_END   = '360000'        # Azimuth window end   (millidegrees)

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

    bringup_launch_dir = Path(get_package_share_directory('my_robot_bringup')) / 'launch'

    # ── Print active subsystems ──────────────────────────────────────────────
    actions.append(LogInfo(msg='--- Sensor Fusion Launch ---'))
    actions.append(LogInfo(msg=f'  LiDAR bringup         : {LAUNCH_LIDAR_BRINGUP}'))
    actions.append(LogInfo(msg=f'  Camera bringup        : {LAUNCH_CAMERA_BRINGUP}'))
    actions.append(LogInfo(msg=f'  Rear camera bringup   : {LAUNCH_REAR_CAMERA_BRINGUP}'))
    actions.append(LogInfo(msg=f'  LiDAR clustering      : {LAUNCH_LIDAR_CLUSTERING}'))
    actions.append(LogInfo(msg=f'  Camera detection      : {LAUNCH_CAMERA_DETECTION}'))
    actions.append(LogInfo(msg=f'  Rear camera detection : {LAUNCH_REAR_CAMERA_DETECTION}'))
    actions.append(LogInfo(msg=f'  Fusion                : {LAUNCH_FUSION}'))

    # ── LiDAR stack ──────────────────────────────────────────────────────────
    if LAUNCH_LIDAR_BRINGUP or LAUNCH_LIDAR_CLUSTERING:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_launch_dir / 'lidar.launch.py')),
            launch_arguments={
                'launch_bringup':    str(LAUNCH_LIDAR_BRINGUP).lower(),
                'launch_clustering': str(LAUNCH_LIDAR_CLUSTERING).lower(),
                'sensor_hostname':   LIDAR_SENSOR_IP,
                'udp_dest':          LIDAR_HOST_IP,
                'azimuth_start':     LIDAR_AZIMUTH_START,
                'azimuth_end':       LIDAR_AZIMUTH_END,
            }.items(),
        ))

    # ── Camera stack ─────────────────────────────────────────────────────────
    if any([LAUNCH_CAMERA_BRINGUP, LAUNCH_REAR_CAMERA_BRINGUP,
            LAUNCH_CAMERA_DETECTION, LAUNCH_REAR_CAMERA_DETECTION]):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_launch_dir / 'cameras.launch.py')),
            launch_arguments={
                'launch_front_bringup':   str(LAUNCH_CAMERA_BRINGUP).lower(),
                'launch_rear_bringup':    str(LAUNCH_REAR_CAMERA_BRINGUP).lower(),
                'launch_front_detection': str(LAUNCH_CAMERA_DETECTION).lower(),
                'launch_rear_detection':  str(LAUNCH_REAR_CAMERA_DETECTION).lower(),
                'front_camera_model':     ZED_CAMERA_MODEL,
                'front_camera_name':      ZED_CAMERA_NAME,
                'front_serial_number':    ZED_SERIAL_NUMBER,
                'rear_camera_model':      ZED_REAR_CAMERA_MODEL,
                'rear_camera_name':       ZED_REAR_CAMERA_NAME,
                'rear_serial_number':     ZED_REAR_SERIAL_NUMBER,
            }.items(),
        ))

    # ── Fusion node ──────────────────────────────────────────────────────────
    if LAUNCH_FUSION:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_launch_dir / 'fusion.launch.py')),
        ))

    return LaunchDescription(actions)
