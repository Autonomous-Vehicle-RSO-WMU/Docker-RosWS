import rclpy
import numpy as np
import  my_robot_cam2points.depend.cam_dependencies as cam_dependencies
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import NavSatFix,PointCloud2,PointField
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D
from yolop_lane_ros2.msg import LaneData
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#These are the functions that will be used by the node to make Camera to Coords.
#The node has not been created to allow me to accomodate to subscribing to the nodes to get depth and lane line data




#assumes left would be negative and is implemented because of the camera being put on an adjustable mount. makes it able to 
#make adjustments based on camera rotation. 

class cam_coordsNode(Node):
    def __init__(self):
        super().__init__('camtocoords')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.lanecoordspublisher_L = self.create_publisher(
            PointCloud2,
            '/relative_lanecoords_L',
            qos_profile
        )
        self.lanecoordspublisher_R = self.create_publisher(
            PointCloud2,
            '/relative_lanecoords_R',
            qos_profile
        )
        self.lanecoordspublisher_M = self.create_publisher(
            PointCloud2,
            '/relative_lanecoords_Center',
            qos_profile
        )
        
        self.pothole = self.create_publisher(
             PointCloud2,
             '/potholes',
             qos_profile
         )

        self.lanedetectionsubscription = self.create_subscription(
            LaneData,          # message type we're receiving
            'overlay_data',         
            self.camLane_callback,  # callback when message arrives
            qos_profile
        )
        

        self.cam2dsubscription=self.create_subscription(
            Detection2D(),
            "/camera/detections2d",
            self.objcallback,
            qos_profile
        )

        self.camDetails={
    # details about the camera
    "camera_focal": 2.8,
    "sensor_height": 3.6,
    "sensor_width": 4.8,
    "image_w": 1920,
    "image_h": 1080,

    # GPS & camera positions (meters)
    "camera_location": np.array([[0.381], [0], [1.01]]),

    "camera_rotation_on_mount":0 ,
    "camera_verticalrotation":0.6283,

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



     
    def camLane_callback(self,msg):
        L=([[],[],[]])
        R=([[],[],[]])
        MID=([[],[],[]])
        N,E,_=cam_dependencies.cam_to_coords(self.camDetails,np.array(msg.left_lane_x),np.array(msg.left_lane_y))
        N2,E2,_=cam_dependencies.cam_to_coords(self.camDetails,np.array(msg.right_lane_x),np.array(msg.right_lane_y)  )
        N3,E3,_=cam_dependencies.cam_to_coords(self.camDetails,np.array(msg.center_x),np.array(msg.center_y)  )
        for x in range (len(N)):
            L[0].append(N[x]+self.camDetails.cameralocation[0][0])
            L[1].append(-E[x])
            L[2].append(0)
        for x in range (len(N2)):
            R[0].append(N2[x]+self.camDetails.cameralocation[0][0])
            R[1].append(-E2[x])
            R[2].append(0)
        for x in range (len(N3)):
            MID[0].append(N3[x]+self.camDetails.cameralocation[0][0])
            MID[1].append(-E3[x])
            MID[2].append(0)
        L= zip(L[0], L[1], L[2])
        R= zip(R[0], R[1], R[2]) 
        MID= zip(MID[0], MID[1], MID[2]) 
        L_cloud=cam_dependencies.make_cloud(self,L)
        R_cloud=cam_dependencies.make_cloud(self,R)
        M_cloud=cam_dependencies.make_cloud(self,MID)
        self.lanecoordspublisher_L.publish(L_cloud)
        self.lanecoordspublisher_R.publish(R_cloud)
        self.lanecoordspublisher_M.publish(M_cloud)


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
