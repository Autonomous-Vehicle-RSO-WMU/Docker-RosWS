import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2  # Standard ROS 2 tool for building point clouds

# --- MESSAGE IMPORTS ---
# NOTE: The standard ROS 2 way to import custom messages is:
# from <your_package_name>.msg import EnrichedClusterArray
# But if your current setup works, you can uncomment your original line below:
from fusion_msgs.msg import EnrichedClusterArray 


class enriched_Translator(Node):
    def __init__(self):
        super().__init__('enriched_translator')

        self.declare_parameter('input_topic', '/fusion/enriched_clusters')
        self.declare_parameter('output_topic', '/ObjectPoints')
        self.declare_parameter('min_lidar_confidence', 0.5)
        self.declare_parameter('min_camera_confidence', 0.5)
        self.declare_parameter('output_frame_id', '')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_lidar_confidence = self.get_parameter('min_lidar_confidence').get_parameter_value().double_value
        self.min_camera_confidence = self.get_parameter('min_camera_confidence').get_parameter_value().double_value
        self.output_frame_id = self.get_parameter('output_frame_id').get_parameter_value().string_value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.object_publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile
        )

        self.fusion_subscription = self.create_subscription(
            EnrichedClusterArray,          
            input_topic,
            self.objcallback,  
            qos_profile
        )
        
    def generate_local_cuboid(self, dimensions, density=30):
        """Generates a point cloud for a cuboid centered at (0,0,0)."""
        L, W, H = dimensions
        
        x_min, x_max = -L/2.0, L/2.0
        y_min, y_max = -W/2.0, W/2.0
        z_min, z_max = -H/2.0, H/2.0
        
        nx = max(2, int(L * density))
        ny = max(2, int(W * density))
        nz = max(2, int(H * density))
        
        x_vals = np.linspace(x_min, x_max, nx)
        y_vals = np.linspace(y_min, y_max, ny)
        z_vals = np.linspace(z_min, z_max, nz)
        
        x_out, y_out, z_out = [], [], []
        
        def append_point(x, y, z):
            x_out.append(x)
            y_out.append(y)
            z_out.append(z)

        # --- Face Pair 1: Front & Back ---
        for y in y_vals:
            for z in z_vals:
                append_point(x_min, y, z)
                append_point(x_max, y, z)
                
        # --- Face Pair 2: Left & Right ---
        for x in x_vals:
            for z in z_vals:
                append_point(x, y_min, z)
                append_point(x, y_max, z)
                
        # --- Face Pair 3: Bottom & Top ---
        for x in x_vals:
            for y in y_vals:
                append_point(x, y, z_min)
                append_point(x, y, z_max)
                
        return np.column_stack([x_out, y_out, z_out])

    def objcallback(self, msg):
        header = msg.header
        if self.output_frame_id:
            header.frame_id = self.output_frame_id

        all_points = []
    
        for cluster in msg.clusters:
            if (
                cluster.lidar_confidence > self.min_lidar_confidence
                and cluster.camera_confidence > self.min_camera_confidence
            ):
                dimensions = np.array(
                    [cluster.dimensions.x, cluster.dimensions.y, cluster.dimensions.z],
                    dtype=np.float32,
                )
                if (not np.all(np.isfinite(dimensions))) or np.any(dimensions <= 0.0):
                    continue
                
                # 1. Get points centered at (0,0,0) -> P_local
                local_points = self.generate_local_cuboid(dimensions, density=30)
                
                # 2. Extract rotation from the quaternion -> R
                quat = np.array([
                    cluster.orientation.x,
                    cluster.orientation.y,
                    cluster.orientation.z,
                    cluster.orientation.w,
                ], dtype=np.float64)
                if (not np.all(np.isfinite(quat))) or np.linalg.norm(quat) < 1e-8:
                    quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
                rotation = R.from_quat(quat)
                
                # 3. Rotate the local points to match real-world yaw/pitch/roll -> R * P_local
                rotated_points = rotation.apply(local_points)
                
                # 4. Translate points to the actual centroid -> + T
                centroid = np.array([cluster.centroid.x, cluster.centroid.y, cluster.centroid.z])
                world_points = rotated_points + centroid
                
                all_points.append(world_points)
        
        # Publish an empty cloud so downstream consumers still receive timely updates.
        if not all_points:
            cloud = pc2.create_cloud_xyz32(header, np.empty((0, 3), dtype=np.float32))
            self.object_publisher.publish(cloud)
            return
            
        # Stack all (N, 3) arrays into a single massive (Total_Points, 3) matrix
        coords = np.ascontiguousarray(np.vstack(all_points), dtype=np.float32)
        
        # Create PointCloud2 directly using standard ROS 2 Python libraries.
        # Passing msg.header ensures the frame_id and timestamp perfectly match the incoming data.
        cloud = pc2.create_cloud_xyz32(header, coords)
        
        self.object_publisher.publish(cloud)


# Entry point of the program
def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create an instance of our node
    node = enriched_Translator()

    # Keep the node alive and processing callbacks
    rclpy.spin(node)

    # Clean up once the node stops
    node.destroy_node()

    # Shut down ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()