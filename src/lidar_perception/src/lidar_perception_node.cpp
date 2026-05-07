#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

// Placeholder-only LiDAR perception node.
// TODO: Implement real clustering and 3D bounding box estimation.

class LidarPerceptionNode : public rclcpp::Node
{
public:
  LidarPerceptionNode()
  : Node("lidar_perception_node")
  {
    // Declare parameters (mirrored in YAML)
    this->declare_parameter<std::string>("frames.lidar_frame", "os_sensor");
    this->declare_parameter<std::string>("frames.output_frame", "lidar_frame");

    this->declare_parameter<std::string>("topics.pointcloud_in", "/ouster/points");
    this->declare_parameter<std::string>("topics.detections3d_out", "/lidar/detections3d");

    this->declare_parameter<bool>("clustering.enabled", true);
    this->declare_parameter<std::string>("clustering.algorithm", "euclidean");
    this->declare_parameter<double>("clustering.cluster_tolerance_m", 0.45);
    this->declare_parameter<int>("clustering.min_cluster_size", 40);
    this->declare_parameter<int>("clustering.max_cluster_size", 20000);

  this->declare_parameter<bool>("ground_filter.enabled", true);
  this->declare_parameter<double>("ground_filter.min_z_m", -0.3);
  this->declare_parameter<double>("ground_filter.max_z_m", 0.2);

    this->declare_parameter<double>("postprocessing.min_distance_m", 0.5);
    this->declare_parameter<double>("postprocessing.max_distance_m", 60.0);
    this->declare_parameter<double>("postprocessing.min_height_m", -2.0);
    this->declare_parameter<double>("postprocessing.max_height_m", 3.0);
    this->declare_parameter<double>("postprocessing.min_bbox_x_m", 0.1);
    this->declare_parameter<double>("postprocessing.max_bbox_x_m", 3.0);
    this->declare_parameter<double>("postprocessing.min_bbox_y_m", 0.1);
    this->declare_parameter<double>("postprocessing.max_bbox_y_m", 3.0);
    this->declare_parameter<double>("postprocessing.min_bbox_z_m", 0.3);
    this->declare_parameter<double>("postprocessing.max_bbox_z_m", 3.0);
    this->declare_parameter<double>("postprocessing.confidence_scale", 1000.0);

    this->declare_parameter<bool>("output.publish_debug_cloud", false);
    this->declare_parameter<std::string>("output.debug_cloud_topic", "/lidar/debug_cloud");
    this->declare_parameter<bool>("output.publish_markers", false);
    this->declare_parameter<std::string>("output.markers_topic", "/lidar/cluster_markers");

    // Load topics
    std::string pointcloud_topic = this->get_parameter("topics.pointcloud_in").as_string();
    std::string detections_topic = this->get_parameter("topics.detections3d_out").as_string();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarPerceptionNode::cloudCallback, this, std::placeholders::_1));

    detections_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
      detections_topic, 10);

    publish_debug_cloud_ = this->get_parameter("output.publish_debug_cloud").as_bool();
    debug_cloud_topic_ = this->get_parameter("output.debug_cloud_topic").as_string();
    if (publish_debug_cloud_) {
      debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_cloud_topic_, rclcpp::SystemDefaultsQoS());
    }

    publish_markers_ = this->get_parameter("output.publish_markers").as_bool();
    markers_topic_ = this->get_parameter("output.markers_topic").as_string();
    if (publish_markers_) {
      markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, 10);
    }

    // Cache all processing parameters at startup
    load_cached_params();

    // Re-cache on dynamic parameter changes
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        load_cached_params();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    RCLCPP_INFO(this->get_logger(), "lidar_perception_node started. Listening on %s, publishing %s",
      pointcloud_topic.c_str(), detections_topic.c_str());
    if (publish_debug_cloud_) {
      RCLCPP_INFO(this->get_logger(), "Debug cloud enabled on %s", debug_cloud_topic_.c_str());
    }
    if (publish_markers_) {
      RCLCPP_INFO(this->get_logger(), "Markers enabled on %s", markers_topic_.c_str());
    }
  }

private:
  void load_cached_params()
  {
    clustering_enabled_ = this->get_parameter("clustering.enabled").as_bool();
    cluster_tolerance_ = this->get_parameter("clustering.cluster_tolerance_m").as_double();
    min_cluster_size_ = this->get_parameter("clustering.min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("clustering.max_cluster_size").as_int();
    min_distance_ = this->get_parameter("postprocessing.min_distance_m").as_double();
    max_distance_ = this->get_parameter("postprocessing.max_distance_m").as_double();
    min_height_ = this->get_parameter("postprocessing.min_height_m").as_double();
    max_height_ = this->get_parameter("postprocessing.max_height_m").as_double();
    min_bbox_x_ = this->get_parameter("postprocessing.min_bbox_x_m").as_double();
    max_bbox_x_ = this->get_parameter("postprocessing.max_bbox_x_m").as_double();
    min_bbox_y_ = this->get_parameter("postprocessing.min_bbox_y_m").as_double();
    max_bbox_y_ = this->get_parameter("postprocessing.max_bbox_y_m").as_double();
    min_bbox_z_ = this->get_parameter("postprocessing.min_bbox_z_m").as_double();
    max_bbox_z_ = this->get_parameter("postprocessing.max_bbox_z_m").as_double();
    confidence_scale_ = this->get_parameter("postprocessing.confidence_scale").as_double();
    ground_filter_enabled_ = this->get_parameter("ground_filter.enabled").as_bool();
    ground_min_z_ = this->get_parameter("ground_filter.min_z_m").as_double();
    ground_max_z_ = this->get_parameter("ground_filter.max_z_m").as_double();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Use cached parameters (updated via on_set_parameters_callback)
    const bool clustering_enabled = clustering_enabled_;
    const double cluster_tolerance = cluster_tolerance_;
    const int min_cluster_size = min_cluster_size_;
    const int max_cluster_size = max_cluster_size_;
    const double min_distance = min_distance_;
    const double max_distance = max_distance_;
    const double min_height = min_height_;
    const double max_height = max_height_;
    const double min_bbox_x = min_bbox_x_;
    const double max_bbox_x = max_bbox_x_;
    const double min_bbox_y = min_bbox_y_;
    const double max_bbox_y = max_bbox_y_;
    const double min_bbox_z = min_bbox_z_;
    const double max_bbox_z = max_bbox_z_;
    const double confidence_scale = confidence_scale_;
    const bool ground_filter_enabled = ground_filter_enabled_;
    const double ground_min_z = ground_min_z_;
    const double ground_max_z = ground_max_z_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud_in);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_filtered->reserve(cloud_in->size());
    for (const auto & pt : cloud_in->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const double distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (distance < min_distance || distance > max_distance) {
        continue;
      }
      if (ground_filter_enabled && pt.z >= ground_min_z && pt.z <= ground_max_z) {
        continue;
      }
      if (pt.z < min_height || pt.z > max_height) {
        continue;
      }
      cloud_filtered->points.push_back(pt);
    }
    cloud_filtered->width = static_cast<uint32_t>(cloud_filtered->points.size());
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = false;

    auto detections = vision_msgs::msg::Detection3DArray();
    detections.header = msg->header;

    if (!clustering_enabled || cloud_filtered->empty()) {
      detections_pub_->publish(detections);
      return;
    }

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (publish_debug_cloud_) {
      debug_cloud->reserve(cloud_filtered->points.size());
    }

    int color_index = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    if (publish_markers_) {
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);
    }
    for (const auto & indices : cluster_indices) {
      float min_x = std::numeric_limits<float>::max();
      float min_y = std::numeric_limits<float>::max();
      float min_z = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float max_y = std::numeric_limits<float>::lowest();
      float max_z = std::numeric_limits<float>::lowest();

      for (const auto idx : indices.indices) {
        const auto & pt = cloud_filtered->points[idx];
        min_x = std::min(min_x, pt.x);
        min_y = std::min(min_y, pt.y);
        min_z = std::min(min_z, pt.z);
        max_x = std::max(max_x, pt.x);
        max_y = std::max(max_y, pt.y);
        max_z = std::max(max_z, pt.z);
      }

      const double cluster_size = static_cast<double>(indices.indices.size());
      const double confidence = std::min(1.0, std::log1p(cluster_size) /
        std::log1p(std::max(1.0, confidence_scale)));

      vision_msgs::msg::Detection3D det;
      det.header = msg->header;
      det.bbox.center.position.x = (min_x + max_x) * 0.5f;
      det.bbox.center.position.y = (min_y + max_y) * 0.5f;
      det.bbox.center.position.z = (min_z + max_z) * 0.5f;
      det.bbox.size.x = std::max(0.01f, max_x - min_x);
      det.bbox.size.y = std::max(0.01f, max_y - min_y);
      det.bbox.size.z = std::max(0.01f, max_z - min_z);

      if (det.bbox.size.x < min_bbox_x || det.bbox.size.x > max_bbox_x ||
          det.bbox.size.y < min_bbox_y || det.bbox.size.y > max_bbox_y ||
          det.bbox.size.z < min_bbox_z || det.bbox.size.z > max_bbox_z) {
        ++color_index;
        continue;
      }
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = "cluster";
      hypothesis.hypothesis.score = confidence;
      det.results.push_back(hypothesis);
      detections.detections.push_back(det);

      if (publish_markers_) {
        const float cx = det.bbox.center.position.x;
        const float cy = det.bbox.center.position.y;
        const float cz = det.bbox.center.position.z;
        const float hx = det.bbox.size.x * 0.5f;
        const float hy = det.bbox.size.y * 0.5f;
        const float hz = det.bbox.size.z * 0.5f;

        std::vector<geometry_msgs::msg::Point> corners(8);
        corners[0].x = cx - hx; corners[0].y = cy - hy; corners[0].z = cz - hz;
        corners[1].x = cx + hx; corners[1].y = cy - hy; corners[1].z = cz - hz;
        corners[2].x = cx + hx; corners[2].y = cy + hy; corners[2].z = cz - hz;
        corners[3].x = cx - hx; corners[3].y = cy + hy; corners[3].z = cz - hz;
        corners[4].x = cx - hx; corners[4].y = cy - hy; corners[4].z = cz + hz;
        corners[5].x = cx + hx; corners[5].y = cy - hy; corners[5].z = cz + hz;
        corners[6].x = cx + hx; corners[6].y = cy + hy; corners[6].z = cz + hz;
        corners[7].x = cx - hx; corners[7].y = cy + hy; corners[7].z = cz + hz;

        const int edges[12][2] = {
          {0,1},{1,2},{2,3},{3,0},
          {4,5},{5,6},{6,7},{7,4},
          {0,4},{1,5},{2,6},{3,7}
        };

        visualization_msgs::msg::Marker wire;
        wire.header = msg->header;
        wire.ns = "clusters_wire";
        wire.id = color_index;
        wire.type = visualization_msgs::msg::Marker::LINE_LIST;
        wire.action = visualization_msgs::msg::Marker::ADD;
        wire.scale.x = 0.03;
        wire.color.r = 0.1f;
        wire.color.g = 0.8f;
        wire.color.b = 0.2f;
        wire.color.a = 0.9f;
        wire.lifetime = rclcpp::Duration::from_seconds(0.1);
        wire.points.reserve(24);
        for (const auto & edge : edges) {
          wire.points.push_back(corners[edge[0]]);
          wire.points.push_back(corners[edge[1]]);
        }
        marker_array.markers.push_back(wire);

        visualization_msgs::msg::Marker text;
        text.header = msg->header;
        text.ns = "clusters_conf";
        text.id = color_index;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = cx;
        text.pose.position.y = cy;
        text.pose.position.z = cz + hz + 0.2f;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.35;
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 0.9f;
        text.lifetime = rclcpp::Duration::from_seconds(0.1);
          std::ostringstream label;
          label << "conf=" << std::fixed << std::setprecision(1) << (confidence * 100.0)
            << "% (n=" << static_cast<int>(cluster_size) << ")";
          text.text = label.str();
        marker_array.markers.push_back(text);
      }

      if (publish_debug_cloud_) {
        const float cluster_id = static_cast<float>(color_index + 1);
        for (const auto idx : indices.indices) {
          auto pt = cloud_filtered->points[idx];
          pt.intensity = cluster_id;
          debug_cloud->points.push_back(pt);
        }
      }

      ++color_index;
    }

    detections_pub_->publish(detections);

    if (publish_markers_) {
      markers_pub_->publish(marker_array);
    }

    if (publish_debug_cloud_) {
      if (debug_cloud->points.empty()) {
        *debug_cloud = *cloud_filtered;
      }
      debug_cloud->width = static_cast<uint32_t>(debug_cloud->points.size());
      debug_cloud->height = 1;
      debug_cloud->is_dense = false;
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*debug_cloud, out_msg);
      out_msg.header = msg->header;
      debug_cloud_pub_->publish(out_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  bool publish_debug_cloud_{false};
  std::string debug_cloud_topic_;
  bool publish_markers_{false};
  std::string markers_topic_;

  // Cached processing parameters (updated via on_set_parameters_callback)
  bool clustering_enabled_{true};
  double cluster_tolerance_{0.45};
  int min_cluster_size_{40};
  int max_cluster_size_{20000};
  double min_distance_{0.5};
  double max_distance_{60.0};
  double min_height_{-2.0};
  double max_height_{3.0};
  double min_bbox_x_{0.1};
  double max_bbox_x_{3.0};
  double min_bbox_y_{0.1};
  double max_bbox_y_{3.0};
  double min_bbox_z_{0.3};
  double max_bbox_z_{3.0};
  double confidence_scale_{1000.0};
  bool ground_filter_enabled_{true};
  double ground_min_z_{-0.3};
  double ground_max_z_{0.2};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarPerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
