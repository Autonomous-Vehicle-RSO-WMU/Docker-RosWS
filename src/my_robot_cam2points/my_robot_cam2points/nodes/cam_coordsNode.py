import rclpy
import numpy as np
import  depend.cam_dependencies as cam_dependencies
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import NavSatFix,PointCloud2,PointField
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D
import struct
from yolop_lane_ros2.msg import LaneData


#These are the functions that will be used by the node to make Camera to Coords.
#The node has not been created to allow me to accomodate to subscribing to the nodes to get depth and lane line data




#assumes left would be negative and is implemented because of the camera being put on an adjustable mount. makes it able to 
#make adjustments based on camera rotation. 

class cam_coordsNode(Node):
    def __init__(self):
        super().__init__('camtocoords')

        self.lanecoordspublisher_L = self.create_publisher(
            PointCloud2,
            '/relative_lanecoords_L',
            10
        )
        self.lanecoordspublisher_L = self.create_publisher(
            PointCloud2,
            '/relative_lanecoords_L',
            10
        )
        
        self.pothole = self.create_publisher(
             PointCloud2,
             'waypoint',
             10
         )
        self.gpscubscription= self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gpscallback,
            10
        )
        self.lanedetectionsubscription = self.create_subscription(
            LaneData,          # message type we're receiving
            'overlay_data',          # no topic name because no working node yet
            self.camLane_callback,  # callback when message arrives
            10                  # queue size / history depth
        )
        

        self.cam2dsubscription=self.create_subscription(
            Detection2D(),
            "/camera/detections2d",
            self.objcallback
        )
   
        self.orientation=np.array[0,0,0]
        
        self.camDetails={
    # details about the camera
    "camera_focal": 2.8,
    "sensor_height": 3.6,
    "sensor_width": 4.8,
    "image_w": 1920,
    "image_h": 1080,

    # GPS & camera positions (meters)
    "base_location": np.array([[0], [0], [0]]),
    "camera_location": np.array([[0.25], [0], [1.01]]),

    # camera distortion constants
    "k1": 0,
    "k2": 0,
    "k3": 0,
    "p1": 0,
    "p2": 0,
    "camera_rotation_on_mount":0 ,
    "camera_verticarotation":0,
    "camera_tilt":0
    }
      


    def objcallback(self,msg):
        potcoords=[[],[]]
        potholes=list(filter(lambda obj: obj.class_id=="pothole", msg.detections))
        pothole_radius=0.6 #meters
        if(len(potholes)>0):
            for pothole in potholes:
                u = pothole.bbox.center.position.x
                v = pothole.bbox.center.position.y
                
                N, E, _ = cam_dependencies.cam_to_coords(self.camDetails, u, v,self.orientation[0],self.orientation[1],self.orientation[2])
                
                angles = np.linspace(0, 2*np.pi, 8)
                for angle in angles:
                    x = N + pothole_radius * np.cos(angle)
                    y = E + pothole_radius * np.sin(angle)
                    potcoords.append([x, y, 0.0])

        cloud = cam_dependencies.make_cloud('potholecoords',potcoords)
        self.pothole_pub.publish(cloud)
        """
       Im just assuming its in a in32 array sorted like this
       [[x1],[y1],[x2],[y2]]
       """



    def Imu_callback(self,msg):
        self.orientation=[msg.x,msg.y.msg.z]      
    def camLane_callback(self,msg):
        L=([[],[],[]])
        R=([[],[],[]])
        N,E,_=cam_dependencies.cam_to_coords(self.camDetails,msg.left_lane_x,msg.left_lane_y,self.orientation[0],self.orientation[1],self.orientation[2])
        N2,E2,_=cam_dependencies.cam_to_coords(self.camDetails,msg.right_lane_x,msg.right_lane_y,self.orientation[0],self.orientation[1],self.orientation[2])
        

        for x in range (len(N)):
            L[0].append(N[x])
            L[1].append(-E[x])
            L[2].append(0)
        for x in range (len(N2)):
            R[0].append(N2[x])
            R[1].append(-E2[x])
            R[2].append(0)
        L= zip(L[0], L[1], L[2])
        R= zip(R[0], R[1], R[2]) 
        L_cloud=cam_dependencies.make_cloud(L)
        R_cloud=cam_dependencies.make_cloud(R)
        self.lanecoordspublisher_L.publish(self,L_cloud)
        self.lanecoordspublisher_R.publish(self,R_cloud)


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
