#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace camping_cart::sensing
{

class RadarCostGridNode : public rclcpp::Node
{
public:
  RadarCostGridNode()
  : Node("radar_cost_grid")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/sensing/radar/near_cost_grid");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 120);
    height_ = declare_parameter<int>("height", 120);
    origin_x_ = declare_parameter<double>("origin_x", -6.0);
    origin_y_ = declare_parameter<double>("origin_y", -6.0);
    free_value_ = declare_parameter<int>("free_value", 0);
    obstacle_value_ = declare_parameter<int>("obstacle_value", 100);
    unknown_value_ = declare_parameter<int>("unknown_value", 0);
    obstacle_radius_m_ = declare_parameter<double>("obstacle_radius_m", 0.30);
    max_message_age_sec_ = declare_parameter<double>("max_message_age_sec", 0.35);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>{
        "/sensing/radar/front/range",
        "/sensing/radar/right1/range",
        "/sensing/radar/right2/range",
        "/sensing/radar/left1/range",
        "/sensing/radar/left2/range",
        "/sensing/radar/rear/range"});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());

    samples_.resize(input_topics_.size());
    for (std::size_t i = 0; i < input_topics_.size(); ++i) {
      subs_.push_back(create_subscription<sensor_msgs::msg::Range>(
        input_topics_[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::Range::ConstSharedPtr msg) { onRange(i, msg); }));
    }

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RadarCostGridNode::publishGrid, this));
  }

private:
  struct RangeSample
  {
    sensor_msgs::msg::Range msg;
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void onRange(std::size_t idx, const sensor_msgs::msg::Range::ConstSharedPtr msg)
  {
    if (!msg || idx >= samples_.size()) {
      return;
    }
    samples_[idx].msg = *msg;
    samples_[idx].recv_time = now();
    samples_[idx].valid = true;
  }

  void markDisk(nav_msgs::msg::OccupancyGrid & grid, double x, double y)
  {
    const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
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
        grid.data[idx] = static_cast<int8_t>(obstacle_value_);
      }
    }
  }

  bool transformHitToBase(
    const sensor_msgs::msg::Range & msg,
    geometry_msgs::msg::PointStamped & hit_base)
  {
    if (!std::isfinite(msg.range)) {
      return false;
    }
    if (msg.range < msg.min_range || msg.range > msg.max_range) {
      return false;
    }
    if (msg.header.frame_id.empty()) {
      return false;
    }

    geometry_msgs::msg::PointStamped hit_sensor;
    hit_sensor.header = msg.header;
    if (hit_sensor.header.stamp.sec == 0 && hit_sensor.header.stamp.nanosec == 0) {
      hit_sensor.header.stamp = now();
    }
    hit_sensor.point.x = msg.range;
    hit_sensor.point.y = 0.0;
    hit_sensor.point.z = 0.0;

    try {
      hit_base = tf_buffer_->transform(
        hit_sensor, base_frame_id_, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "radar_cost_grid TF transform failed (%s -> %s): %s",
        hit_sensor.header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
      return false;
    }
  }

  void publishGrid()
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = base_frame_id_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    const int initial_value = (unknown_value_ >= -1 && unknown_value_ <= 100) ?
      unknown_value_ : free_value_;
    grid.data.assign(static_cast<std::size_t>(width_ * height_), static_cast<int8_t>(initial_value));

    const auto now_time = now();
    for (const auto & sample : samples_) {
      if (!sample.valid) {
        continue;
      }
      if ((now_time - sample.recv_time).seconds() > max_message_age_sec_) {
        continue;
      }

      geometry_msgs::msg::PointStamped hit_base;
      if (!transformHitToBase(sample.msg, hit_base)) {
        continue;
      }
      markDisk(grid, hit_base.point.x, hit_base.point.y);
    }

    pub_grid_->publish(grid);
  }

  std::string output_topic_;
  std::string base_frame_id_;
  double resolution_{0.10};
  int width_{120};
  int height_{120};
  double origin_x_{-6.0};
  double origin_y_{-6.0};
  int free_value_{0};
  int obstacle_value_{100};
  int unknown_value_{0};
  double obstacle_radius_m_{0.30};
  double max_message_age_sec_{0.35};
  double publish_rate_hz_{10.0};
  std::vector<std::string> input_topics_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> subs_;
  std::vector<RangeSample> samples_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camping_cart::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::sensing::RadarCostGridNode>());
  rclcpp::shutdown();
  return 0;
}
