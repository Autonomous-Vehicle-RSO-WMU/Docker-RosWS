#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # --- Get GPS Target Waypoints ---
    # Create temporary helper node to use params
    param_node = rclpy.create_node('waypoint_follower_params')
    # Declare parameter for target waypoints (flattened array of floats: (lat1, long1, lat2, long2))
    param_node.declare_parameter('target_waypoints', [0.0, 0.0, 0.0, 0.0])
    # Get target waypoints from mission_config.yaml
    flat_waypoints = param_node.get_parameter('target_waypoints').get_parameter_value().double_array_value
    # Repackage flat array into coordinate pairs
    target_gps_waypoints = [(flat_waypoints[i], flat_waypoints[i+1]) for i in range(0, len(flat_waypoints), 2)]
    # OBLITERATE the node
    param_node.destroy_node()

    # --- Wait for navigation to fully activate ---
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    # --- Translate Raw GPS waypoints ---
    # Create temporary node & client to handle translation services
    translator_node = rclpy.create_node('waypoint_translator')
    from_ll_client = translator_node.create_client(FromLL, '/fromLL')
    while not from_ll_client.wait_for_service(timeout_sec=1.0):
        translator_node.get_logger().info('/fromLL service not available, waiting again...')

    # Apply points to poseStamped msgs
    waypoints = []

    points_pose = PoseStamped()
    points_pose.header.frame_id = 'map'
    points_pose.header.stamp = navigator.get_clock().now().to_msg()
    points_pose.pose.orientation.w = 1.0  # Use "Identity" quaternion, does not matter (see goal_yaw_tolerance in nav2 config)                    
    for pt in target_gps_waypoints:
        request = FromLL.Request()
        request.ll_point.latitude = float(pt[0])
        request.ll_point.longitude = float(pt[1])
        request.ll_point.altitude = 0.0

        future = from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(translator_node, future)
        map_point = future.result().map_point

        points_pose.pose.position.x = map_point.x
        points_pose.pose.position.y = map_point.y
        waypoints.append(deepcopy(points_pose))

    nav_start = navigator.get_clock().now()

    # Start task of navigation through waypoints
    nav_through_poses_task = navigator.goThroughPoses(waypoints)

    # TODO: Hardcode specific shared and/or indvidual route behavior for each course here? Or is this different concept or should be done elsewhere
    while not navigator.isTaskComplete(task=nav_through_poses_task) and rclpy.ok():
        time.sleep(1) # Doesn't halt navigation

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print(f'Goal failed!{error_code}:{error_msg}')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()