/**
 * pointcloud_merger_node.cpp
 *
 * Merges Ouster LiDAR (XYZI) and ZED stereo depth (XYZRGB) point clouds
 * into a single XYZI PointCloud2 in a common frame.
 *
 * ZED clouds are:
 *   1. Transformed into the output frame via TF
 *   2. Converted from XYZRGB → XYZI (intensity = luminance)
 *   3. Voxel-downsampled to reduce density
 *   4. Concatenated with the LiDAR cloud
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>


class PointCloudMergerNode : public rclcpp::Node
{
public:
  PointCloudMergerNode()
  : Node("pointcloud_merger_node")
  {
    // ---------- Parameters ----------
    declare_parameter("output_frame", "os_sensor");
    declare_parameter("output_topic", "/merged/points");
    declare_parameter("lidar_topic", "/ouster/points");
    declare_parameter("lidar_enabled", true);
    declare_parameter("camera_front_topic", "/zed/zed_node/point_cloud/cloud_registered");
    declare_parameter("camera_front_enabled", true);
    declare_parameter("camera_front_voxel_leaf_m", 0.05);
    declare_parameter("camera_rear_topic", "/zed_rear/zed_node/point_cloud/cloud_registered");
    declare_parameter("camera_rear_enabled", true);
    declare_parameter("camera_rear_voxel_leaf_m", 0.05);
    declare_parameter("max_stale_sec", 1.0);
    declare_parameter("rate_hz", 10.0);
    declare_parameter("tf_timeout_sec", 1.0);

    output_frame_         = get_parameter("output_frame").as_string();
    max_stale_sec_        = get_parameter("max_stale_sec").as_double();
    front_voxel_leaf_     = get_parameter("camera_front_voxel_leaf_m").as_double();
    rear_voxel_leaf_      = get_parameter("camera_rear_voxel_leaf_m").as_double();
    tf_timeout_sec_       = get_parameter("tf_timeout_sec").as_double();
    double rate_hz        = get_parameter("rate_hz").as_double();

    // ---------- TF ----------
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------- QoS ----------
    // BestEffort for LiDAR (Ouster uses BestEffort)
    auto qos_be = rclcpp::QoS(5)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);
    // Reliable for ZED depth clouds (ZED wrapper publishes Reliable by default)
    auto qos_rel = rclcpp::QoS(5)
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);

    // ---------- Publisher ----------
    auto out_topic = get_parameter("output_topic").as_string();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10);

    // ---------- Subscribers ----------
    if (get_parameter("lidar_enabled").as_bool()) {
      auto topic = get_parameter("lidar_topic").as_string();
      sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos_be,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latest_lidar_ = msg;
          lidar_stamp_  = this->now();
        });
      RCLCPP_INFO(get_logger(), "Subscribed to LiDAR: %s", topic.c_str());
    }

    if (get_parameter("camera_front_enabled").as_bool()) {
      auto topic = get_parameter("camera_front_topic").as_string();
      sub_cam_front_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos_rel,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latest_cam_front_ = msg;
          cam_front_stamp_  = this->now();
        });
      RCLCPP_INFO(get_logger(), "Subscribed to front camera cloud: %s", topic.c_str());
    }

    if (get_parameter("camera_rear_enabled").as_bool()) {
      auto topic = get_parameter("camera_rear_topic").as_string();
      sub_cam_rear_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos_rel,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latest_cam_rear_ = msg;
          cam_rear_stamp_  = this->now();
        });
      RCLCPP_INFO(get_logger(), "Subscribed to rear camera cloud: %s", topic.c_str());
    }

    // ---------- Timer ----------
    int period_ms = static_cast<int>(1000.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PointCloudMergerNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "PointCloud merger ready — output frame: %s, rate: %.0f Hz",
      output_frame_.c_str(), rate_hz);
  }

private:
  // ----------------------------------------------------------------
  // Timer callback: merge and publish
  // ----------------------------------------------------------------
  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mtx_);

    auto now = this->now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
    rclcpp::Time best_stamp = now;
    bool have_any = false;
    size_t lidar_pts = 0, front_pts = 0, rear_pts = 0;

    // 1. LiDAR cloud — already in output frame (os_sensor), just convert
    if (latest_lidar_ && !is_stale(lidar_stamp_, now)) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_pcl(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*latest_lidar_, *lidar_pcl);
      lidar_pts = lidar_pcl->size();
      *merged += *lidar_pcl;
      best_stamp = rclcpp::Time(latest_lidar_->header.stamp);
      have_any = true;
    }

    // 2. Front camera cloud
    if (latest_cam_front_ && !is_stale(cam_front_stamp_, now)) {
      auto cam_xyzi = process_camera_cloud(latest_cam_front_, front_voxel_leaf_);
      if (cam_xyzi && !cam_xyzi->empty()) {
        front_pts = cam_xyzi->size();
        *merged += *cam_xyzi;
        have_any = true;
      }
    }

    // 3. Rear camera cloud
    if (latest_cam_rear_ && !is_stale(cam_rear_stamp_, now)) {
      auto cam_xyzi = process_camera_cloud(latest_cam_rear_, rear_voxel_leaf_);
      if (cam_xyzi && !cam_xyzi->empty()) {
        rear_pts = cam_xyzi->size();
        *merged += *cam_xyzi;
        have_any = true;
      }
    }

    if (!have_any) {
      return;
    }

    // Use wall clock for stamp if no LiDAR data yet
    if (lidar_pts == 0) {
      best_stamp = now;
    }

    // Publish merged cloud
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*merged, out_msg);
    out_msg.header.frame_id = output_frame_;
    out_msg.header.stamp = best_stamp;
    pub_->publish(out_msg);

    // Throttled diagnostic log — shows per-source point counts and ages
    merge_count_++;
    auto now_sec = now.seconds();
    if (now_sec - last_log_sec_ > 5.0) {
      last_log_sec_ = now_sec;
      double lidar_age = latest_lidar_ ? (now - lidar_stamp_).seconds() : -1.0;
      double front_age = latest_cam_front_ ? (now - cam_front_stamp_).seconds() : -1.0;
      double rear_age  = latest_cam_rear_ ? (now - cam_rear_stamp_).seconds() : -1.0;
      RCLCPP_INFO(get_logger(),
        "Merged %zu pts | lidar=%zu (%.2fs) front=%zu (%.2fs) rear=%zu (%.2fs) | merges=%d",
        merged->size(), lidar_pts, lidar_age, front_pts, front_age, rear_pts, rear_age,
        merge_count_);
    }
  }

  // ----------------------------------------------------------------
  // Process a ZED camera cloud: TF transform → XYZRGB→XYZI → voxel downsample
  // ----------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr process_camera_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    double voxel_leaf)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);

    // Transform cloud to output frame
    sensor_msgs::msg::PointCloud2 transformed;
    try {
      auto tf = tf_buffer_->lookupTransform(
        output_frame_, cloud_msg->header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      tf2::doTransform(*cloud_msg, transformed, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF transform failed for %s: %s",
        cloud_msg->header.frame_id.c_str(), ex.what());
      return result;
    }

    // Convert XYZRGB → XYZI
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(transformed, *rgb_cloud);

    result->reserve(rgb_cloud->size());
    for (const auto & pt : rgb_cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      pcl::PointXYZI pi;
      pi.x = pt.x;
      pi.y = pt.y;
      pi.z = pt.z;
      // Luminance from RGB
      pi.intensity = 0.299f * pt.r + 0.587f * pt.g + 0.114f * pt.b;
      result->push_back(pi);
    }

    // Voxel downsample
    if (voxel_leaf > 0.0 && !result->empty()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(result);
      float leaf = static_cast<float>(voxel_leaf);
      vg.setLeafSize(leaf, leaf, leaf);
      vg.filter(*downsampled);
      return downsampled;
    }

    return result;
  }

  // ----------------------------------------------------------------
  bool is_stale(const rclcpp::Time & stamp, const rclcpp::Time & now) const
  {
    return (now - stamp).seconds() > max_stale_sec_;
  }

  // ----------------------------------------------------------------
  // Members
  // ----------------------------------------------------------------
  std::string output_frame_;
  double max_stale_sec_;
  double front_voxel_leaf_;
  double rear_voxel_leaf_;
  double tf_timeout_sec_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam_front_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam_rear_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cam_front_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cam_rear_;
  rclcpp::Time lidar_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cam_front_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cam_rear_stamp_{0, 0, RCL_ROS_TIME};

  int merge_count_ = 0;
  double last_log_sec_ = 0.0;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
