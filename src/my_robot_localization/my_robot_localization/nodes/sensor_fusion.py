#!/usr/bin/env python3
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/carla/hero/lidar')
        self.radar_sub = message_filters.Subscriber(self, PointCloud2, '/carla/hero/radar_front')
        self.fused_pub = self.create_publisher(PointCloud2, '/carla/hero/fused_objects', 10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.radar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.fuse_callback)

    def fuse_callback(self, lidar_msg, radar_msg):
        lidar_points = list(point_cloud2.read_points(lidar_msg, field_names=('x','y','z'), skip_nans=True))
        radar_points = list(point_cloud2.read_points(radar_msg, field_names=('x','y','z'), skip_nans=True))
        fused_points = lidar_points + radar_points
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = lidar_msg.header.frame_id or 'map'
        fused_cloud = point_cloud2.create_cloud_xyz32(header, fused_points)
        self.fused_pub.publish(fused_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
