#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace camping_cart::sensing
{

class LidarCostGridNode : public rclcpp::Node
{
public:
  LidarCostGridNode()
  : Node("lidar_cost_grid")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/perception/obstacles");
    output_topic_ = declare_parameter<std::string>("output_topic", "/sensing/lidar/near_cost_grid");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 160);
    height_ = declare_parameter<int>("height", 160);
    origin_x_ = declare_parameter<double>("origin_x", -8.0);
    origin_y_ = declare_parameter<double>("origin_y", -8.0);
    free_value_ = declare_parameter<int>("free_value", 0);
    unknown_value_ = declare_parameter<int>("unknown_value", -1);
    min_cost_ = declare_parameter<int>("min_cost", 35);
    max_cost_ = declare_parameter<int>("max_cost", 100);
    cost_range_min_m_ = declare_parameter<double>("cost_range_min_m", 0.4);
    cost_range_max_m_ = declare_parameter<double>("cost_range_max_m", 8.0);
    obstacle_radius_m_ = declare_parameter<double>("obstacle_radius_m", 0.20);
    ego_clear_radius_m_ = declare_parameter<double>("ego_clear_radius_m", 0.90);
    max_message_age_sec_ = declare_parameter<double>("max_message_age_sec", 0.50);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarCostGridNode::onCloud, this, std::placeholders::_1));

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LidarCostGridNode::publishGrid, this));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    latest_cloud_ = msg;
    latest_cloud_rx_ = now();
  }

  static double squaredDistance2d(
    const double x1, const double y1,
    const double x2, const double y2)
  {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return dx * dx + dy * dy;
  }

  int mapDistanceToCost(const double distance_m) const
  {
    const double max_cost_range = std::max(cost_range_min_m_ + 1e-3, cost_range_max_m_);
    if (distance_m <= cost_range_min_m_) {
      return max_cost_;
    }
    if (distance_m >= max_cost_range) {
      return min_cost_;
    }
    const double norm =
      (distance_m - cost_range_min_m_) / (max_cost_range - cost_range_min_m_);
    const double inv = 1.0 - std::clamp(norm, 0.0, 1.0);
    return static_cast<int>(std::round(
      static_cast<double>(min_cost_) +
      inv * static_cast<double>(max_cost_ - min_cost_)));
  }

  void markDisk(
    nav_msgs::msg::OccupancyGrid & grid,
    const double grid_origin_x,
    const double grid_origin_y,
    const double x,
    const double y,
    const int value)
  {
    const int cx = static_cast<int>(std::floor((x - grid_origin_x) / resolution_));
    const int cy = static_cast<int>(std::floor((y - grid_origin_y) / resolution_));
    const int radius_cells =
      static_cast<int>(std::ceil(std::max(0.0, obstacle_radius_m_) / resolution_));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
          continue;
        }
        const std::size_t idx = static_cast<std::size_t>(gy * width_ + gx);
        if (grid.data[idx] < 0) {
          grid.data[idx] = static_cast<int8_t>(value);
        } else {
          grid.data[idx] = static_cast<int8_t>(std::max<int>(grid.data[idx], value));
        }
      }
    }
  }

  void clearDisk(
    nav_msgs::msg::OccupancyGrid & grid,
    const double grid_origin_x,
    const double grid_origin_y,
    const double x,
    const double y)
  {
    const int cx = static_cast<int>(std::floor((x - grid_origin_x) / resolution_));
    const int cy = static_cast<int>(std::floor((y - grid_origin_y) / resolution_));
    const int radius_cells =
      static_cast<int>(std::ceil(std::max(0.0, ego_clear_radius_m_) / resolution_));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
          continue;
        }
        const std::size_t idx = static_cast<std::size_t>(gy * width_ + gx);
        grid.data[idx] = static_cast<int8_t>(free_value_);
      }
    }
  }

  bool getBasePoseInOutput(geometry_msgs::msg::PointStamped & base_in_output)
  {
    geometry_msgs::msg::PointStamped base_origin;
    base_origin.header.stamp = now();
    base_origin.header.frame_id = base_frame_id_;
    base_origin.point.x = 0.0;
    base_origin.point.y = 0.0;
    base_origin.point.z = 0.0;

    try {
      base_in_output = tf_buffer_->transform(
        base_origin, output_frame_id_, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lidar_cost_grid failed to locate %s in %s: %s",
        base_frame_id_.c_str(), output_frame_id_.c_str(), ex.what());
      return false;
    }
  }

  bool getCloudTransform(
    const std_msgs::msg::Header & cloud_header,
    geometry_msgs::msg::TransformStamped & tf_out)
  {
    if (cloud_header.frame_id.empty()) {
      return false;
    }

    auto stamp = cloud_header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = now();
    }

    try {
      tf_out = tf_buffer_->lookupTransform(
        output_frame_id_, cloud_header.frame_id, stamp, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lidar_cost_grid TF lookup failed (%s -> %s): %s",
        cloud_header.frame_id.c_str(), output_frame_id_.c_str(), ex.what());
      return false;
    }
  }

  void publishGrid()
  {
    geometry_msgs::msg::PointStamped base_in_output;
    if (!getBasePoseInOutput(base_in_output)) {
      return;
    }

    const double grid_origin_x = base_in_output.point.x + origin_x_;
    const double grid_origin_y = base_in_output.point.y + origin_y_;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = output_frame_id_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);
    grid.info.origin.position.x = grid_origin_x;
    grid.info.origin.position.y = grid_origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    const int initial_value = (unknown_value_ >= -1 && unknown_value_ <= 100) ?
      unknown_value_ : free_value_;
    grid.data.assign(static_cast<std::size_t>(width_ * height_), static_cast<int8_t>(initial_value));

    if (!latest_cloud_ || (now() - latest_cloud_rx_).seconds() > max_message_age_sec_) {
      pub_grid_->publish(grid);
      return;
    }

    geometry_msgs::msg::TransformStamped cloud_tf_msg;
    if (!getCloudTransform(latest_cloud_->header, cloud_tf_msg)) {
      pub_grid_->publish(grid);
      return;
    }

    tf2::Transform cloud_tf;
    tf2::fromMsg(cloud_tf_msg.transform, cloud_tf);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        const double z = static_cast<double>(*iter_z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const tf2::Vector3 p_sensor(x, y, z);
        const tf2::Vector3 p_output = cloud_tf * p_sensor;
        const double distance = std::sqrt(squaredDistance2d(
          p_output.x(), p_output.y(),
          base_in_output.point.x, base_in_output.point.y));
        const int value = mapDistanceToCost(distance);
        markDisk(
          grid, grid_origin_x, grid_origin_y,
          p_output.x(), p_output.y(), value);
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lidar_cost_grid failed to iterate point cloud: %s", ex.what());
    }

    if (ego_clear_radius_m_ > 0.0) {
      clearDisk(
        grid, grid_origin_x, grid_origin_y,
        base_in_output.point.x, base_in_output.point.y);
    }

    pub_grid_->publish(grid);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  double resolution_{0.10};
  int width_{160};
  int height_{160};
  double origin_x_{-8.0};
  double origin_y_{-8.0};
  int free_value_{0};
  int unknown_value_{-1};
  int min_cost_{35};
  int max_cost_{100};
  double cost_range_min_m_{0.4};
  double cost_range_max_m_{8.0};
  double obstacle_radius_m_{0.20};
  double ego_clear_radius_m_{0.90};
  double max_message_age_sec_{0.50};
  double publish_rate_hz_{10.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  rclcpp::Time latest_cloud_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camping_cart::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camping_cart::sensing::LidarCostGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
