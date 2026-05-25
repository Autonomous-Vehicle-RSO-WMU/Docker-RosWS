import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField



#sample dict format
details = {
    # details about the camera
    "camera_focal": 2.8,
    "sensor_height": 3.6,
    "sensor_width": 4.8,
    "image_w": 1280,
    "image_h": 720,

    # GPS & camera positions (meters)
    #front, side to side, height
    "gps_location": np.array([[0], [0], [0]]),
    "camera_location": np.array([[0], [0], [1.106]]),
    "camera_verticalrotation":30,

}



def cam_to_coords(details,u,v):
    x_prime,y_prime=camera_to_normalized(u,v,details)
   
    NED=convert_to_North_East_Down(x_prime,y_prime,details)
  
    return(NED)
    
#convert back to normalized 
def camera_to_normalized(u,v,details ):
    fx=(details["camera_focal"]/details["sensor_width"])*details["image_w"]
    fy=(details["camera_focal"]/details["sensor_height"])*details["image_h"]
    
##coordinates of the middle point (0,0) equivalent if it were a grid
    cx=details["image_w"]/2
    cy=details["image_h"]/2

    x_prime=(u-cx)/fx
    y_prime=(v-cy)/fy


    return x_prime,y_prime



def convert_to_North_East_Down(x_prime,y_prime,details):
# 1. Undistort and 
    
    # 2. Camera Mounting Geometry
    # H: Height of camera lens above the ground plane
    H = details['camera_location'][2][0] 
    
    # theta: The downward tilt of the camera relative to the vehicle's floor
    theta = np.radians(details['camera_verticalrotation'])
    
    # 3. Calculate Forward Distance (Relative North)
    # alpha: The pixel's angle relative to the camera center
    alpha_y = np.arctan(y_prime)
    print(theta)
    print(alpha_y)
    phi = theta + alpha_y
    
   # This is the horizontal distance from the pole to the point
    rel_north = H / np.tan(phi)
    
    # 3. Horizontal Triangle (The "Side" distance)
    # The actual 'forward' distance along the camera's tilted floor-ray 
    # is rel_north / cos(alpha_y).
    # But simplified: rel_east is just x_prime scaled by the distance 
    # from the lens to the vertical plane the point sits on.
    
    rel_east = x_prime * rel_north
    
    # 5. Add Mounting Offsets
    # If camera_location is [x_offset, y_offset, height]
    N = rel_north 
    E = rel_east 
    D = H # The point is on the ground
    return([N,E,D])





def make_cloud(node,points):
    cloud = PointCloud2()
    cloud.header.frame_id='base_link'
    cloud.header.stamp = node.get_clock().now().to_msg()
    cloud.height = 1
    cloud.width = len(list(points))
    cloud.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(list(points))
    cloud.data = b''.join(struct.pack('fff', x, y, z) for x, y, z in points)
    return cloud
