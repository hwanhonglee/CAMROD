#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <string>

/**
 * LiDAR preprocessor node.
 * 역할: 각도 필터 + ROI 박스 + Voxel Downsampling
 * 흐름: points_raw -> (이 노드) -> filtered_cloud -> ground_segmentation_ros2
 *
 * HJ_260623: ground_segmentation_ros2 앞단 전처리로 원본 포인트를 줄여
 * ground_seg/perception 부하를 낮춘다.
 */
class LidarPreprocessorNode : public rclcpp::Node {
public:
  LidarPreprocessorNode() : Node("lidar_preprocessor") {
    this->declare_parameter<std::string>("input_topic", "vanjee/points_raw");
    this->declare_parameter<std::string>("output_topic", "filtered_cloud");
    this->declare_parameter<std::string>("marker_frame_id", "lidar_link");

    this->declare_parameter("angle_filter_deg", 64.0);
    this->declare_parameter("roi_x_min",  0.0);
    this->declare_parameter("roi_x_max",  3.0);
    this->declare_parameter("roi_y_min", -1.5);
    this->declare_parameter("roi_y_max",  1.5);
    this->declare_parameter("roi_z_min", -1.0);
    this->declare_parameter("roi_z_max",  1.0);
    this->declare_parameter("voxel_leaf_size", 0.03);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();

    angle_filter_rad_ = this->get_parameter("angle_filter_deg").as_double() * M_PI / 180.0;
    this->get_parameter("roi_x_min", roi_x_min_);
    this->get_parameter("roi_x_max", roi_x_max_);
    this->get_parameter("roi_y_min", roi_y_min_);
    this->get_parameter("roi_y_max", roi_y_max_);
    this->get_parameter("roi_z_min", roi_z_min_);
    this->get_parameter("roi_z_max", roi_z_max_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

    voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

    // ground_segmentation_ros2 subscribes reliably; keep this edge reliable too.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&LidarPreprocessorNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      qos);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("roi_marker", 10);
    marker_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LidarPreprocessorNode::publishRoiMarker, this));

    RCLCPP_INFO(this->get_logger(), "[camrod] LiDAR preprocessor started");
    RCLCPP_INFO(
      this->get_logger(),
      "in=%s out=%s frame=%s | angle: +/-%.0f deg | ROI X[%.1f~%.1f] Y[%.1f~%.1f] Z[%.1f~%.1f] | voxel: %.2fm",
      input_topic_.c_str(), output_topic_.c_str(), marker_frame_id_.c_str(),
      this->get_parameter("angle_filter_deg").as_double(),
      roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_,
      voxel_leaf_size_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());

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

      const double azimuth = std::atan2(y, x);
      if (std::fabs(azimuth) > angle_filter_rad_) {
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
      filtered->push_back(pt);
    }

    if (filtered->empty()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_filter_.setInputCloud(filtered);
    voxel_filter_.filter(*downsampled);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*downsampled, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);

    RCLCPP_DEBUG(
      this->get_logger(), "filtered: %zu -> voxel: %zu",
      filtered->size(), downsampled->size());
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
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;
  double voxel_leaf_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
