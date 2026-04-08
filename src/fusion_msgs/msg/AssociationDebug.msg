# Debug information for tuning association parameters

std_msgs/Header header

# Association information arrays (parallel arrays)
int32[] lidar_cluster_ids
int32[] camera_detection_ids
float32[] iou_2d_scores
float32[] distance_3d_scores
float32[] centroid_pixel_distances
bool[] associated
string[] rejection_reasons

# Summary statistics
int32 total_lidar_clusters
int32 total_camera_detections
int32 successful_associations
int32 unmatched_lidar_clusters
int32 unmatched_camera_detections

