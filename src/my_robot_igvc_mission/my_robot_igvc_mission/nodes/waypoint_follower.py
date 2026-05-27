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

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from robot_localization.srv import FromLL


class NavsatReadinessMonitor:

    def __init__(self):
        self.received_odometry_gps = False

    def odometry_gps_callback(self, _msg):
        self.received_odometry_gps = True

class waypoint_params(Node):

    def __init__(self):
        super().__init__('waypoint_parms')

        print("======== START DIAGNOSTICS IN waypoint_params ========\n")

        # Declare parameter for target waypoints with fallback (flattened array of floats: (lat1, long1, lat2, long2))
        self.declare_parameter('target_waypoints', [0.0, 0.0, 0.0, 0.0])

        # Retrieve target waypoints from mission_config.yaml
        self.flat_waypoints = self.get_parameter('target_waypoints').get_parameter_value().double_array_value
        print("self.flat_waypoints: ", self.flat_waypoints , "\n")
      
    
    def initialize_waypoints(self):
        
        # Repackage flat array into coordinate pairs
        target_gps_waypoints = [(self.flat_waypoints[i], self.flat_waypoints[i+1]) for i in range(0, len(self.flat_waypoints), 2)]
        print("target_gps_waypoints: ", target_gps_waypoints , "\n")

        print("======== END DIAGNOSTICS IN waypoint_params ========\n")
        return target_gps_waypoints

def gps_to_pose(node, target_gps_waypoints):

    # Create temporary node & client to handle translation services
    from_ll_client = node.create_client(FromLL, '/fromLL')
    while not from_ll_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('/fromLL service not available, waiting again...')

    readiness_monitor = NavsatReadinessMonitor()
    odometry_gps_sub = node.create_subscription(
        Odometry,
        'odometry/gps',
        readiness_monitor.odometry_gps_callback,
        10,
    )
    while rclpy.ok() and not readiness_monitor.received_odometry_gps:
        node.get_logger().info('Waiting for odometry/gps before requesting /fromLL conversions...')
        rclpy.spin_once(node, timeout_sec=1.0)

    # Apply points to poseStamped msgs
    waypoints = []
    i = 0
                    
    for pt in target_gps_waypoints:
        print("======== START DIAGNOSTICS IN gps_to_pose.target_gps_waypoints ========\n")
        points_pose = PoseStamped()
        points_pose.header.frame_id = 'map'
        points_pose.header.stamp = node.get_clock().now().to_msg()
        points_pose.pose.orientation.w = 1.0  # Use "Identity" quaternion, does not matter (see goal_yaw_tolerance in nav2 config)

        request = FromLL.Request()
        print("Point ", i+1, " LAT: ", pt[0], "\n")
        request.ll_point.latitude = float(pt[0])
        print("Point ", i+1, " LON: ", pt[1], "\n")
        request.ll_point.longitude = float(pt[1])
        request.ll_point.altitude = 0.0

        future = from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response is None:
            raise RuntimeError(f'/fromLL request failed for waypoint {i + 1}')

        map_point = response.map_point
        if map_point.x == 0.0 and map_point.y == 0.0 and (request.ll_point.latitude != 0.0 or request.ll_point.longitude != 0.0):
            raise RuntimeError(
                f'/fromLL returned an uninitialized map point for waypoint {i + 1}. '
                'navsat_transform is likely not ready yet.'
            )

        print("map.point.x for point ", i+1, " ", map_point.x, "\n")
        points_pose.pose.position.x = map_point.x
        print("points_pose.pose.position.x for point ", i+1, " ", points_pose.pose.position.x, "\n")

        print("map.point.y for point ", i+1, " ", map_point.y, "\n")
        points_pose.pose.position.y = map_point.y
        print("points_pose.pose.position.y for point ", i+1, " ", points_pose.pose.position.y, "\n")

        print("points_pose ", i+1, " ", points_pose.pose.position.x, "\n")
        waypoints.append(points_pose)
        print("waypoints for iteration on point ", i+1, " ", waypoints, "\n")

        
        i = i+1
        print("======== END DIAGNOSTICS IN gps_to_pose ========\n")

    node.destroy_subscription(odometry_gps_sub)
    return waypoints



def main():
    rclpy.init()

    navigator = BasicNavigator()

    # ========================
    # Get GPS Target Waypoints
    # ========================
    params = waypoint_params()
    target_gps_waypoints = params.initialize_waypoints()
    
    # =====================================
    # Wait for navigation to fully activate
    # =====================================
    navigator.waitUntilNav2Active(localizer='robot_localization')

    # ===========================
    # Translate Raw GPS waypoints
    # ===========================
    waypoints = gps_to_pose(navigator, target_gps_waypoints)

    # ========================================
    # Start task to navigate through waypoints
    # ========================================
    nav_through_poses_task = navigator.goThroughPoses(waypoints)

    # TODO: Hardcode specific shared and/or indvidual route behavior for each course here? Or is this different concept or should be done elsewhere
    while not navigator.isTaskComplete() and rclpy.ok():
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
