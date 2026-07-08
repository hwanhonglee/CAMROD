#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

/**
 * LiDAR preprocessor node.
 * Role: angle filter + ROI crop + voxel downsampling.
 * Flow: points_raw -> this node -> filtered_cloud -> ground_segmentation_ros2.
 *
 * HJ_260623: Reduce raw points before ground_segmentation_ros2 to lower
 * ground_seg/perception load.
 */
class LidarPreprocessorNode : public rclcpp::Node {
public:
  LidarPreprocessorNode()
      : Node("lidar_preprocessor"),
        filtered_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
        downsampled_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()) {
    this->declare_parameter<std::string>("input_topic", "vanjee/points_raw");
    this->declare_parameter<std::string>("output_topic", "filtered_cloud");
    this->declare_parameter<std::string>("marker_frame_id", "lidar_link");

    this->declare_parameter("angle_filter_deg", 64.0);
    this->declare_parameter("roi_x_min", 0.0);
    this->declare_parameter("roi_x_max", 3.0);
    this->declare_parameter("roi_y_min", -1.5);
    this->declare_parameter("roi_y_max", 1.5);
    this->declare_parameter("roi_z_min", -1.0);
    this->declare_parameter("roi_z_max", 1.0);
    this->declare_parameter("voxel_leaf_size", 0.03);
    // HH_260707: Keep functionality unchanged by default; operators can cap
    // preprocessing only when raw LiDAR arrives faster than downstream can use.
    this->declare_parameter("max_process_hz", 0.0);
    this->declare_parameter("qos_depth", 2);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();

    angle_filter_rad_ =
        this->get_parameter("angle_filter_deg").as_double() * M_PI / 180.0;
    angle_filter_tan_ = std::tan(angle_filter_rad_);
    this->get_parameter("roi_x_min", roi_x_min_);
    this->get_parameter("roi_x_max", roi_x_max_);
    this->get_parameter("roi_y_min", roi_y_min_);
    this->get_parameter("roi_y_max", roi_y_max_);
    this->get_parameter("roi_z_min", roi_z_min_);
    this->get_parameter("roi_z_max", roi_z_max_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("max_process_hz", max_process_hz_);
    const int qos_depth = std::max(
        1, static_cast<int>(this->get_parameter("qos_depth").as_int()));

    voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_,
                              voxel_leaf_size_);

    // HH_260707: ground_segmentation_ros2 subscribes reliably; keep reliability
    // but use a shallow latest-only queue to avoid stale PointCloud2 backlog.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        std::bind(&LidarPreprocessorNode::callback, this,
                  std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_,
                                                                 qos);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "roi_marker", 10);
    marker_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LidarPreprocessorNode::publishRoiMarker, this));

    RCLCPP_INFO(this->get_logger(), "[camrod] LiDAR preprocessor started");
    RCLCPP_INFO(
        this->get_logger(),
        "in=%s out=%s frame=%s | angle: +/-%.0f deg | ROI X[%.1f~%.1f] "
        "Y[%.1f~%.1f] Z[%.1f~%.1f] | voxel: %.2fm | qos_depth=%d max_hz=%.1f",
        input_topic_.c_str(), output_topic_.c_str(), marker_frame_id_.c_str(),
        this->get_parameter("angle_filter_deg").as_double(), roi_x_min_,
        roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_,
        voxel_leaf_size_, qos_depth, max_process_hz_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (max_process_hz_ > 0.0) {
      const auto now_time = now();
      if (last_process_time_.nanoseconds() != 0 &&
          (now_time - last_process_time_).seconds() < (1.0 / max_process_hz_)) {
        return;
      }
      last_process_time_ = now_time;
    }

    filtered_->clear();
    downsampled_->clear();
    filtered_->reserve(static_cast<std::size_t>(msg->width) * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // HH_260707: Equivalent forward-cone check without per-point atan2().
      if (x <= 0.0f ||
          std::fabs(y) > (static_cast<double>(x) * angle_filter_tan_)) {
        continue;
      }

      if (x < roi_x_min_ || x > roi_x_max_) {
        continue;
      }
      if (y < roi_y_min_ || y > roi_y_max_) {
        continue;
      }
      if (z < roi_z_min_ || z > roi_z_max_) {
        continue;
      }

      pcl::PointXYZI pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      pt.intensity = 0.0f;
      filtered_->push_back(pt);
    }

    if (filtered_->empty()) {
      return;
    }

    voxel_filter_.setInputCloud(filtered_);
    voxel_filter_.filter(*downsampled_);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*downsampled_, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);

    RCLCPP_DEBUG(this->get_logger(), "filtered: %zu -> voxel: %zu",
                 filtered_->size(), downsampled_->size());
  }

  void publishRoiMarker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "roi_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (roi_x_min_ + roi_x_max_) / 2.0;
    marker.pose.position.y = (roi_y_min_ + roi_y_max_) / 2.0;
    marker.pose.position.z = (roi_z_min_ + roi_z_max_) / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = roi_x_max_ - roi_x_min_;
    marker.scale.y = roi_y_max_ - roi_y_min_;
    marker.scale.z = roi_z_max_ - roi_z_min_;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2f;
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    marker_pub_->publish(marker);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_frame_id_;
  double angle_filter_rad_;
  double angle_filter_tan_;
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;
  double voxel_leaf_size_;
  double max_process_hz_{0.0};
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
