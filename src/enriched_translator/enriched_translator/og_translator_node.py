from fusion_msgs.msg import EnrichedCluster,EnrichedClusterArray

import rclpy
import numpy as np
from my_robot_cam2points.depend.cam_dependencies import make_cloud
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy




class enriched_Translator(Node):
    def __init__(self):
        super().__init__('enriched_translator')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.object_publisher = self.create_publisher(
            PointCloud2,
            '/ObjectPoints',
            qos_profile
        )

        self.fusion_subscription = self.create_subscription(
            EnrichedClusterArray,          
            "/fusion/enriched_clusters",         
            self.objcallback,  
            qos_profile
        )
    def generate_cuboid_surface_lists(centroid, dimensions, density=30):
        cx, cy, cz = centroid
        L, W, H = dimensions
        
        # 1. Calculate absolute boundaries directly from centroid
        x_min, x_max = cx - L/2.0, cx + L/2.0
        y_min, y_max = cy - W/2.0, cy + W/2.0
        z_min, z_max = cz - H/2.0, cz + H/2.0
        
        # 2. Determine number of sample steps per axis
        nx = max(2, int(L * density))
        ny = max(2, int(W * density))
        nz = max(2, int(H * density))
        
        # 3. Create the linear coordinate values
        x_vals = np.linspace(x_min, x_max, nx)
        y_vals = np.linspace(y_min, y_max, ny)
        z_vals = np.linspace(z_min, z_max, nz)
        
        # Initialize your 3 explicit coordinate trackers
        x_out = []
        y_out = []
        z_out = []
        
        # Helper to append to all three lists simultaneously
        def append_point(x, y, z):
            x_out.append(x)
            y_out.append(y)
            z_out.append(z)

        # --- Face Pair 1: Front & Back (Fix X, sweep Y and Z) ---
        for y in y_vals:
            for z in z_vals:
                append_point(x_min, y, z) # Back Face
                append_point(x_max, y, z) # Front Face
                
        # --- Face Pair 2: Left & Right (Fix Y, sweep X and Z) ---
        for x in x_vals:
            for z in z_vals:
                append_point(x, y_min, z) # Left Face
                append_point(x, y_max, z) # Right Face
                
        # --- Face Pair 3: Bottom & Top (Fix Z, sweep X and Y) ---
        for x in x_vals:
            for y in y_vals:
                append_point(x, y, z_min) # Bottom Face
                append_point(x, y, z_max) # Top Face

        # 4. Pack into your required [[x], [y], [z]] structure
        nested_list = [x_out, y_out, z_out]
        
    
        return nested_list
    def objcallback(self,msg):
        coords=[[],[],[]]
    
        for cluster in msg.clusters:
            if cluster.lidar_confidence > 0.5 and cluster.camera_confidence > 0.5:  # Only consider clusters with high confidence
                centroid = [cluster.centroid.x, cluster.centroid.y, cluster.centroid.z]
                dimensions = [cluster.dimensions.x, cluster.dimensions.y, cluster.dimensions.z]
                dense_point_cloud = generate_cuboid_surface_lists(centroid, dimensions, density=30)
                coords[0]+=dense_point_cloud[0]
                coords[1]+=dense_point_cloud[1]
                coords[2]+=dense_point_cloud[2]     
        coords=np.column_stack(coords)       
        cloud = make_cloud(self,coords,'os_lidar')
        self.object_publisher.publish(cloud)
        
       

        


        

            


       
        


# Entry point of the program
def main():
    # Initialize the ROS 2 system
    rclpy.init()

    # Create an instance of our node
    node = enriched_Translator()

    # Keep the node alive and processing callbacks
    rclpy.spin(node)

    # Clean up once the node stops
    node.destroy_node()

    # Shut down ROS 2
    rclpy.shutdown()


# Only run main() if this file is executed directly
if __name__ == '__main__':
    main()