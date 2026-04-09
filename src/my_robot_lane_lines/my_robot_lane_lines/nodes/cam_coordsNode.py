import rclpy
import numpy as np
import  my_robot_lane_lines.dependency.cam_dependencies as cam_dependencies
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import NavSatFix,PointCloud2,PointField
from std_msgs.msg import Float32

#from yolop_lane_ros2.msg import LaneData

#These are the functions that will be used by the node to make Camera to Coords.
#The node has not been created to allow me to accomodate to subscribing to the nodes to get depth and lane line data






#assumes left would be negative and is implemented because of the camera being put on an adjustable mount. makes it able to 
#make adjustments based on camera rotation. 

class cam_coordsNode(Node):
    def __init__(self):
        super().__init__('camtocoords')

        
        self.lanecoordspublisher = self.create_publisher(
            PointCloud2,
            'relative_lanecoords',
            10
        )
        # self.lanedetectionsubscription = self.create_subscription(
        #     LaneData,          # message type we're receiving
        #     'overlay_data',          # no topic name because no working node yet
        #     self.camLane_callback,  # callback when message arrives
        #     10                  # queue size / history depth
        # )
        
        self.imusubscription=self.create_subscription(
            Float32,
            '/imu/rpy',
            self.imu_callback,
            10
        )
   
        self.orientation=np.array([0,0,0])
        
        self.camDetails={
    # details about the camera
    "camera_focal": 2.8,
    "sensor_height": 3.6,
    "sensor_width": 4.8,
    "image_w": 1920,
    "image_h": 1080,

    # GPS & camera positions (meters)
    "gps_location": np.array([[0], [0], [0]]),
    "camera_location": np.array([[1], [0.5], [0]]),

    # camera distortion constants
    "k1": 0,
    "k2": 0,
    "k3": 0,
    "p1": 0,
    "p2": 0,
    }
      


    


  
        

    
    
    # def camObj_callback(self,msg):
    #     """
    #     Im just assuming its in a in32 array sorted like this
    #     [[x1],[y1],[x2],[y2]]
    #     """
    #     if(len(msg[0])):
    #         self.objectsDetected = np.empty((4, 0))
    #     else:
    #         self.objectsDetected=np.array(msg)


    def imu_callback(self,msg):
        self.orientation=[msg.x,msg.y.msg.z]      
    def camLane_callback(self,msg):
        lanes=[[],[],[]]
        Depth=1 #setting a temp value for now as not finalized where recieved from 
        N,E,D=cam_dependencies.cam_to_coords(self.camDetails,msg.left_lane_x,msg.left_lane_y,Depth,0,0,0,camera_rotation_on_mount=0 ,camera_vertical_rotation=0,camera_tilt=0)
        N2,E2,D2=cam_dependencies.cam_to_coords(self.camDetails,msg.right_lane_x,msg.right_lane_y,Depth,0,0,0,camera_rotation_on_mount=0 ,camera_vertical_rotation=0,camera_tilt=0)
        N+=N2
        E+=E2
        D+=D2

        for x in range (len(N)):
            lanes[0].append(N[x])
            lanes[1].append(-E[x])
            lanes[2].append(D[x])
        
        
        field_x = PointField()
        field_x.name = 'x'
        field_x.offset = 0
        field_x.datatype = PointField.FLOAT32
        field_x.count = 1

        field_y = PointField()
        field_y.name = 'y'
        field_y.offset = 4
        field_y.datatype = PointField.FLOAT32
        field_y.count = 1

        field_z = PointField()
        field_z.name = 'z'
        field_z.offset = 8
        field_z.datatype = PointField.FLOAT32
        field_z.count = 1
        binary_data = b''
        for x, y, z in zip(lanes[0], lanes[1], lanes[2]):
            binary_data += struct.pack('fff', x, y, z)

        # 3. Build the PointCloud2 object
        cloud = PointCloud2()
        cloud.header.frame_id = 'base_link'
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1                        # unordered cloud
        cloud.width = len(lanes)
        cloud.fields = [field_x, field_y, field_z]  # <-- exactly what you said
        cloud.is_bigendian = False
        cloud.point_step = 12                   # 3 fields x 4 bytes each
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = binary_data
        cloud.is_dense = True
        self.lanecoordspublisher.publish(cloud)
        #here should send any data that may be requires as well as updating the local coords stored


# Entry point of the program
def main():
    # Initialize the ROS 2 system
    rclpy.init()

    # Create an instance of our node
    node = cam_coordsNode()

    # Keep the node alive and processing callbacks
    rclpy.spin(node)

    # Clean up once the node stops
    node.destroy_node()

    # Shut down ROS 2
    rclpy.shutdown()


# Only run main() if this file is executed directly
if __name__ == '__main__':
    main()
