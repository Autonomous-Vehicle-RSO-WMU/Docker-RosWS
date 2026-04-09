import numpy as np 
import cv2
# import pandas as pd



#sample dict format
details = {
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



def create_matrixes(details,vehicle_heading,vehicle_slant,vehicle_tilt,camera_rotation_on_mount,camera_vertical_rotation,camera_tilt,relativeto='base'):
    #im assuming that the inputs will be in degrees, if the input is in radianss, this segment will be removed
    vehicle_heading = np.radians(vehicle_heading)
    vehicle_slant   = np.radians(vehicle_slant)
    vehicle_tilt    = np.radians(vehicle_tilt)
    camera_rotation_on_mount  = np.radians(camera_rotation_on_mount)
    camera_vertical_rotation = np.radians(camera_vertical_rotation)
    camera_tilt               = np.radians(camera_tilt)

    # Combined angles
    y_heading = camera_rotation_on_mount + vehicle_heading
    vertical  = vehicle_slant + camera_vertical_rotation
    tilt      = vehicle_tilt + camera_tilt
    #rotation matrix to be able to project based on rotation of the camera 
    #used because in practice the vehicle wilal use GPS
    
    #Yaw
    R   = np.array([
        [-np.sin(y_heading), np.cos(y_heading), 0],
        [0, 0, 1],
        [np.cos(y_heading), np.sin(y_heading), 0]
    ])


    #Pitch
    R_z = np.array([
        [np.cos(vertical), -np.sin(vertical), 0],
        [np.sin(vertical),  np.cos(vertical), 0],
        [0, 0, 1]
    ])

    #Roll
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(tilt), -np.sin(tilt)],
        [0, np.sin(tilt),  np.cos(tilt)]
    ])

    # Offset matrix
    camera_offset= details.camera_location- details.gps_location if relativeto=='gps' else [[0],[0],[0 ]]
    camera_offset_matrix =  _camera_offset_matrix(vehicle_slant, vehicle_heading, vehicle_tilt)
    camera_offset_matrix = camera_offset_matrix @ camera_offset
    R_inv, R_z_inv, R_x_inv = np.linalg.inv(R), np.linalg.inv(R_z), np.linalg.inv(R_x)
    return R, R_inv, R_x_inv, R_z_inv, camera_offset_matrix

    

def _camera_offset_matrix(vehicle_slant, vehicle_heading, vehicle_tilt):
    # Returns 3x3 GPS offset matrix
    return np.array([
        [
            np.cos(vehicle_slant)*np.cos(vehicle_heading),
            np.cos(vehicle_slant)*np.sin(vehicle_heading)*np.sin(vehicle_tilt) - np.sin(vehicle_slant)*np.cos(vehicle_tilt),
            np.cos(vehicle_slant)*np.sin(vehicle_heading)*np.cos(vehicle_tilt) + np.sin(vehicle_slant)*np.sin(vehicle_tilt)
        ],
        [
            np.sin(vehicle_slant)*np.cos(vehicle_heading),
            np.sin(vehicle_slant)*np.sin(vehicle_heading)*np.sin(vehicle_tilt) + np.cos(vehicle_slant)*np.cos(vehicle_tilt),
            np.sin(vehicle_slant)*np.sin(vehicle_heading)*np.cos(vehicle_tilt) - np.cos(vehicle_slant)*np.sin(vehicle_tilt)
        ],
        [
            -np.sin(vehicle_heading),
            np.cos(vehicle_heading)*np.sin(vehicle_tilt),
            np.cos(vehicle_heading)*np.cos(vehicle_tilt)
        ]
    ])


# def cam_to_gps(details,u,v,D,vehicle_heading=0,vehicle_slant=0,vehicle_tilt=0,camera_rotation_on_mount=0 ,camera_vertical_rotation=0,camera_tilt=0):

#     N,E,D,=cam_to_coords(details,u,v,D,vehicle_heading,vehicle_slant,vehicle_tilt,camera_rotation_on_mount,camera_vertical_rotation,camera_tilt,'gps')
    
#     #GPS conversion
    
#     # convert ENU -> geodetic
#     lat, lon, alt = pm.enu2geodetic(N,E,D, lat0, lon0, alt0)
#     print(lat0,lon0,alt0)
#     print(lat, lon, alt)
#     return (lat,lon,alt)
def cam_to_coords(details,u,v,D,vehicle_heading=0,vehicle_slant=0,vehicle_tilt=0,camera_rotation_on_mount=0,camera_vertical_rotation=0,camera_tilt=0,relativeto='base'):
    x_prime,y_prime=camera_to_normalized(u,v,details)
    R,R_inv,R_x_inv,R_z_inv,camera_offsetmatrix=create_matrixes(details,vehicle_heading,vehicle_slant,vehicle_tilt,camera_rotation_on_mount,camera_vertical_rotation,camera_tilt,relativeto)
    NE=convert_to_North_East_Down(x_prime,y_prime,D,R,R_inv,R_x_inv,R_z_inv)
   
    N=NE[0]-camera_offsetmatrix[2]
    E=NE[1]+camera_offsetmatrix[0]

    D-=camera_offsetmatrix[1]
  
    return(N,E,D)
    
#convert back to normalized 
def camera_to_normalized(u,v,details ):
    x_doubleprime=(u-details.cx)/details.fx
    y_doubleprime=(v-details.cy)/details.fy

#undo distortion(if the distortion is removed by the subsystems group already however, this part can be removed)
    distortion = np.array([details.k1, details.k2, details.p1, details.p2, details.k3])
    K=np.array([
        [details.fx,0,details.cx],
        [0,details.fy,details.cy],
        [0,0,1]
    ])

    points = np.stack([x_doubleprime, y_doubleprime], axis=1).astype(np.float32).reshape(-1, 1, 2)                      # shape (N, 1, 2)

    undistorted=cv2.undistortPoints(points,K,distortion,P=K)
    

    #undistorted normalized coordinates
    x_prime = undistorted[:, 0, 0]   
    y_prime = undistorted[:, 0, 1]
    return x_prime,y_prime



def convert_to_North_East_Down(x_prime,y_prime,D,R,R_inv,R_x_inv,R_z_inv):
    zeros=np.zeros(len(x_prime))
    #Yc is made under the assumption as of now that D is constant, D will later be changed when accessing lidar camera
    Yc=(R @ np.array([zeros,zeros,D]))[1]

    #Zc is found intuitively since y'=Yc/Zc so can get y'/Yc= 1/Zc so Zc= Yc/y'
    Zc=(Yc/y_prime)
    Xc=(Zc*x_prime)#inverse of x'=Xc/Zc


    #R is inversed here to be able to do the opposite of distance to camera
  


    # empty zeros is used instead of Yc as down is already known so no need to recompute it 
    c=np.array([
        Xc,
        zeros,
        Zc
    ])
    c=R_x_inv @ c 
    c=R_z_inv @ c

     #getting make the North East Down Matrix 
    c=R_inv @ c
    return c


# u=np.array([455.798365672726,480,460])
# v= np.array([335.93520795940526,220,340])
# D=np.array([0.958309-0.092417]*len(u))

# print("CAM TO COORDS")
# cam_to_coords(u,v,D,0,0,0)
