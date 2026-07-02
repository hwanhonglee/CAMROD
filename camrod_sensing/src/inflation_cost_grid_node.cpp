// HH_260424: Renamed from local_cost_grid_node. Merges all cost sources
//   (lanelet centerline, LiDAR, Radar, global_path) into one robot-centred
//   /planning/cost_grid/inflation. Each output cell takes the max cost across
//   all fresh inputs. Used by the Nav2 local costmap and cmd_vel_gate.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <avg_msgs/msg/occupancy_grid.hpp>
#include <avg_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace camrod::sensing
{

class InflationCostGridNode : public rclcpp::Node
{
public:
  InflationCostGridNode()
  : Node("inflation_cost_grid")
  {
    output_topic_        = declare_parameter<std::string>("output_topic", "/planning/cost_grid/inflation");
    base_frame_id_       = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    output_frame_id_     = declare_parameter<std::string>("output_frame_id", "map");
    resolution_          = declare_parameter<double>("resolution", 0.10);
    width_               = declare_parameter<int>("width", 120);
    height_              = declare_parameter<int>("height", 120);
    origin_x_            = declare_parameter<double>("origin_x", -6.0);
    origin_y_            = declare_parameter<double>("origin_y", -6.0);
    // HH_260618: Keep the OccupancyGrid map-axis aligned for Nav2/gate
    // consumers, but mask merged costs by robot-frame extents so rear and
    // side cost visualization/control load do not grow unnecessarily.
    enable_robot_frame_window_ = declare_parameter<bool>("enable_robot_frame_window", true);
    forward_extent_m_    = declare_parameter<double>("forward_extent_m", 3.0);
    rear_extent_m_       = declare_parameter<double>("rear_extent_m", 1.2);
    side_extent_m_       = declare_parameter<double>("side_extent_m", 1.4);
    free_value_          = declare_parameter<int>("free_value", 0);
    // HH_260422: ego_clear_radius clears the robot footprint so the planner start cell is never lethal.
    ego_clear_radius_m_  = declare_parameter<double>("ego_clear_radius_m", 0.50);
    max_message_age_s_ = declare_parameter<double>("max_message_age_s", 0.50);
    // HH_260618: Default to 6 Hz to match Nav2 local costmap cadence and reduce merge load.
    publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 6.0);

    // HH_260424: Default inputs — lanelet centerline, lidar, radar, global_path route bias.
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>{
        "/map/cost_grid/lanelet",
        "/sensing/cost_grid/lidar",
        "/sensing/cost_grid/radar",
        "/planning/cost_grid/global_path"});

    // HH_260422: Per-input max age override. lanelet / global_path update slowly;
    //   sensor grids update at ~10-20 Hz. If shorter than input list, global fallback is used.
    input_max_ages_s_ = declare_parameter<std::vector<double>>(
      "input_max_ages_s",
      std::vector<double>{5.0, 0.50, 0.50, 10.0});

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_grid_ = create_publisher<avg_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());

    const std::size_t n = input_topics_.size();
    input_grids_.resize(n);
    input_recv_times_.resize(n, rclcpp::Time(0, 0, RCL_ROS_TIME));

    for (std::size_t i = 0; i < n; ++i) {
      // HH_260422: Use VOLATILE durability so subscriptions are compatible with both
      //   TRANSIENT_LOCAL publishers (sensor grids) and VOLATILE publishers (planning grids).
      auto qos = rclcpp::QoS(1).reliable();
      subs_.push_back(create_subscription<avg_msgs::msg::OccupancyGrid>(
        input_topics_[i], qos,
        [this, i](avg_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          input_grids_[i]      = msg;
          input_recv_times_[i] = now();
        }));
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&InflationCostGridNode::publishGrid, this));

    RCLCPP_INFO(get_logger(),
      "inflation_cost_grid ready: %zu inputs → %s  grid=%dx%d res=%.2fm window=%s front=%.2fm rear=%.2fm side=%.2fm",
      n, output_topic_.c_str(), width_, height_, resolution_,
      enable_robot_frame_window_ ? "robot_frame" : "grid_full",
      forward_extent_m_, rear_extent_m_, side_extent_m_);
  }

private:
  // Returns the cost at world position (wx, wy) from a grid, or free_value_ if outside bounds.
  int sampleGrid(
    const avg_msgs::msg::OccupancyGrid & grid,
    double wx, double wy) const
  {
    const double rx = wx - grid.info.origin.position.x;
    const double ry = wy - grid.info.origin.position.y;
    if (rx < 0.0 || ry < 0.0) return free_value_;
    const int cx = static_cast<int>(rx / grid.info.resolution);
    const int cy = static_cast<int>(ry / grid.info.resolution);
    const int w  = static_cast<int>(grid.info.width);
    const int h  = static_cast<int>(grid.info.height);
    if (cx < 0 || cy < 0 || cx >= w || cy >= h) return free_value_;
    const int8_t v = grid.data[static_cast<std::size_t>(cy * w + cx)];
    return (v < 0) ? free_value_ : static_cast<int>(v);
  }

  // Clears a disk of radius ego_clear_radius_m_ around the robot centre.
  void clearEgo(
    avg_msgs::msg::OccupancyGrid & grid,
    double base_x, double base_y) const
  {
    const int cx = static_cast<int>((base_x - grid.info.origin.position.x) / resolution_);
    const int cy = static_cast<int>((base_y - grid.info.origin.position.y) / resolution_);
    const int r  = static_cast<int>(std::ceil(ego_clear_radius_m_ / resolution_));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) continue;
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) continue;
        grid.data[static_cast<std::size_t>(gy * width_ + gx)] =
          static_cast<int8_t>(free_value_);
      }
    }
  }

  void publishGrid()
  {
    geometry_msgs::msg::TransformStamped base_in_output;
    try {
      base_in_output = tf_buffer_->lookupTransform(
        output_frame_id_, base_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "inflation_cost_grid: TF %s→%s failed: %s",
        base_frame_id_.c_str(), output_frame_id_.c_str(), ex.what());
      return;
    }

    const double base_x        = base_in_output.transform.translation.x;
    const double base_y        = base_in_output.transform.translation.y;
    tf2::Quaternion base_orientation;
    tf2::fromMsg(base_in_output.transform.rotation, base_orientation);
    double base_roll = 0.0;
    double base_pitch = 0.0;
    double base_yaw = 0.0;
    tf2::Matrix3x3(base_orientation).getRPY(base_roll, base_pitch, base_yaw);
    const double cos_base_yaw = std::cos(base_yaw);
    const double sin_base_yaw = std::sin(base_yaw);
    const double grid_origin_x = base_x + origin_x_;
    const double grid_origin_y = base_y + origin_y_;

    avg_msgs::msg::OccupancyGrid out;
    out.header.stamp              = now();
    out.header.frame_id           = output_frame_id_;
    out.info.map_load_time        = out.header.stamp;
    out.info.resolution           = static_cast<float>(resolution_);
    out.info.width                = static_cast<uint32_t>(width_);
    out.info.height               = static_cast<uint32_t>(height_);
    out.info.origin.position.x    = grid_origin_x;
    out.info.origin.position.y    = grid_origin_y;
    out.info.origin.position.z    = 0.0;
    out.info.origin.orientation.w = 1.0;
    out.data.assign(
      static_cast<std::size_t>(width_ * height_),
      static_cast<int8_t>(free_value_));

    const auto now_time = now();

    for (int gy = 0; gy < height_; ++gy) {
      for (int gx = 0; gx < width_; ++gx) {
        const double wx = grid_origin_x + (static_cast<double>(gx) + 0.5) * resolution_;
        const double wy = grid_origin_y + (static_cast<double>(gy) + 0.5) * resolution_;
        if (!insideRobotFrameWindow(wx, wy, base_x, base_y, cos_base_yaw, sin_base_yaw)) {
          continue;
        }
        int best = free_value_;

        for (std::size_t i = 0; i < input_grids_.size(); ++i) {
          if (!input_grids_[i]) continue;
          // HH_260422: Use per-input age limit when configured; fall back to global limit.
          const double age_limit =
            (i < input_max_ages_s_.size() && input_max_ages_s_[i] > 0.0)
            ? input_max_ages_s_[i]
            : max_message_age_s_;
          if ((now_time - input_recv_times_[i]).seconds() > age_limit) continue;
          const int c = sampleGrid(*input_grids_[i], wx, wy);
          if (c > best) best = c;
        }

        out.data[static_cast<std::size_t>(gy * width_ + gx)] =
          static_cast<int8_t>(best);
      }
    }

    if (ego_clear_radius_m_ > 0.0) {
      clearEgo(out, base_x, base_y);
    }

    pub_grid_->publish(out);
  }

  bool insideRobotFrameWindow(
    double wx,
    double wy,
    double base_x,
    double base_y,
    double cos_base_yaw,
    double sin_base_yaw) const
  {
    if (!enable_robot_frame_window_) return true;
    const double dx = wx - base_x;
    const double dy = wy - base_y;
    const double local_x = cos_base_yaw * dx + sin_base_yaw * dy;
    const double local_y = -sin_base_yaw * dx + cos_base_yaw * dy;
    return (
      local_x <= forward_extent_m_ &&
      local_x >= -rear_extent_m_ &&
      std::abs(local_y) <= side_extent_m_);
  }

  std::string output_topic_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  double resolution_{0.10};
  int    width_{120};
  int    height_{120};
  double origin_x_{-6.0};
  double origin_y_{-6.0};
  bool   enable_robot_frame_window_{true};
  double forward_extent_m_{3.0};
  double rear_extent_m_{1.2};
  double side_extent_m_{1.4};
  int    free_value_{0};
  double ego_clear_radius_m_{0.50};
  double max_message_age_s_{0.50};
  double publish_rate_hz_{10.0};
  std::vector<std::string> input_topics_;
  std::vector<double>      input_max_ages_s_;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<avg_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::OccupancyGrid>::SharedPtr> subs_;
  std::vector<avg_msgs::msg::OccupancyGrid::ConstSharedPtr> input_grids_;
  std::vector<rclcpp::Time> input_recv_times_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::sensing::InflationCostGridNode>());
  rclcpp::shutdown();
  return 0;
}
