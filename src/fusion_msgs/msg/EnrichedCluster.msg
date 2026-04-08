# Single object cluster with fused LiDAR + Camera data

std_msgs/Header header

# Geometry from LiDAR
geometry_msgs/Point centroid
geometry_msgs/Vector3 dimensions
geometry_msgs/Quaternion orientation

# Classification from Camera
string object_class
float32 camera_confidence
bool camera_validated

# LiDAR features
int32 point_count
float32 lidar_confidence
float32 average_distance

# Fusion metadata
float32 fusion_confidence
int32 cluster_id
float64 association_score

# Optional: 2D bounding box in image
float32 bbox_2d_xmin
float32 bbox_2d_ymin
float32 bbox_2d_xmax
float32 bbox_2d_ymax

