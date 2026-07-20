#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// HH_260720 - Use generated CAMROD range/grid contracts and ROS geometry only for TF.
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_sensing_radar.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <avg_msgs/msg/avg_occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/avg_range.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "camrod_sensing/route_lanelet_cost_filter.hpp"

namespace camrod::sensing
{

class RadarCostGridNode : public rclcpp::Node
{
public:
  using AvgSensingRadar = avg_msgs::msg::AvgSensingRadar;

  // Implements `RadarCostGridNode` behavior.
  RadarCostGridNode()
  : Node("radar_cost_grid")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/sensing/cost_grid/radar");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 120);
    height_ = declare_parameter<int>("height", 120);
    origin_x_ = declare_parameter<double>("origin_x", -6.0);
    origin_y_ = declare_parameter<double>("origin_y", -6.0);
    free_value_ = declare_parameter<int>("free_value", 0);
    unknown_value_ = declare_parameter<int>("unknown_value", -1);
    min_cost_ = declare_parameter<int>("min_cost", 85);
    max_cost_ = declare_parameter<int>("max_cost", 100);
    cost_range_min_m_ = declare_parameter<double>("cost_range_min_m", 0.3);
    // HH_260625: Optional SEN0592 self-echo reject. Cost range min is a max-cost
    // knee, not an ignore threshold, so very small fixed echoes need this gate.
    ignore_below_range_m_ = declare_parameter<double>("ignore_below_range_m", 0.0);
    // HH_260703 - Optional per-topic override, ordered like input_topics.
    // Use this to keep side sensors more sensitive than front/rear self echoes.
    ignore_below_ranges_m_ = declare_parameter<std::vector<double>>(
      "ignore_below_ranges_m", std::vector<double>{});
    // HH_260422: Default lowered to 2.0m — radar is near-field only; per-sensor max_range in Range
    //   message limits detections before they reach the cost mapping stage.
    cost_range_max_m_ = declare_parameter<double>("cost_range_max_m", 2.0);
    obstacle_radius_m_ = declare_parameter<double>("obstacle_radius_m", 0.30);
    // HH_260422: Reduced from 0.90 to 0.50 so near-field side/rear radar readings
    //   (0.5–0.9 m from centre) are not erased by the ego-footprint clear disk.
    ego_clear_radius_m_ = declare_parameter<double>("ego_clear_radius_m", 0.50);
    max_message_age_s_ = declare_parameter<double>("max_message_age_s", 0.35);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    radar_status_topic_ = declare_parameter<std::string>(
      "radar_status_topic", "/sensing/radar/status");
    publish_radar_status_ = declare_parameter<bool>("publish_radar_status", false);
    // HH_260720 - Apply the same active-route obstacle corridor to radar and
    // LiDAR so the safety gate and merged planning grid cannot disagree.
    route_lanelet_filter_enable_ =
      declare_parameter<bool>("route_lanelet_filter_enable", true);
    route_lanelet_mask_topic_ = declare_parameter<std::string>(
      "route_lanelet_mask_topic", "/map/cost_grid/route_lanelet_mask");
    route_lanelet_margin_m_ = declare_parameter<double>("route_lanelet_margin_m", 0.35);
    route_lanelet_allowed_max_cost_ =
      declare_parameter<int>("route_lanelet_allowed_max_cost", 50);
    route_lanelet_mask_max_age_s_ =
      declare_parameter<double>("route_lanelet_mask_max_age_s", 2.5);
    route_lanelet_filter_fail_open_when_robot_outside_ = declare_parameter<bool>(
      "route_lanelet_filter_fail_open_when_robot_outside", true);
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      // HH_260623 - Use the latest 7-channel radar topic set from todo/camrod_sensing.
      std::vector<std::string>{
        "/sensing/radar/front1/range",
        "/sensing/radar/front2/range",
        "/sensing/radar/left1/range",
        "/sensing/radar/left2/range",
        "/sensing/radar/right1/range",
        "/sensing/radar/right2/range",
        "/sensing/radar/rear/range"});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // HH_260720 - Publish radar cost data on the generated CAMROD grid contract.
    pub_grid_ = create_publisher<avg_msgs::msg::AvgOccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());
    avg_radar_pub_ = create_publisher<AvgSensingRadar>(radar_status_topic_, rclcpp::QoS(10));
    // HH_260720 - Receive the latest active-route mask across startup order.
    route_lanelet_mask_sub_ = create_subscription<avg_msgs::msg::AvgOccupancyGrid>(
      route_lanelet_mask_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr msg) {
        onRouteLaneletMask(msg);
      });

    samples_.resize(input_topics_.size());
    for (std::size_t i = 0; i < input_topics_.size(); ++i) {
      subs_.push_back(create_subscription<avg_msgs::msg::AvgRange>(
        input_topics_[i], rclcpp::SensorDataQoS(),
        [this, i](avg_msgs::msg::AvgRange::ConstSharedPtr msg) { onRange(i, msg); }));
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
    avg_msgs::msg::AvgRange msg;
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  // Handles the `onRange` callback.
  void onRange(std::size_t idx, const avg_msgs::msg::AvgRange::ConstSharedPtr msg)
  {
    if (!msg || idx >= samples_.size()) {
      return;
    }
    samples_[idx].msg = *msg;
    samples_[idx].recv_time = now();
    samples_[idx].valid = true;
  }

  // HH_260720 - Cache route-mask validity once per map update.
  void onRouteLaneletMask(const avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    route_lanelet_mask_ = msg;
    route_lanelet_mask_receive_time_ = now();
    route_lanelet_mask_has_allowed_cells_ =
      route_lanelet_cost_filter::hasAllowedCell(*msg, route_lanelet_allowed_max_cost_);
  }

  bool shouldApplyRouteLaneletFilter(
    const geometry_msgs::msg::PointStamped & base_in_output, const rclcpp::Time & now_time)
  {
    if (!route_lanelet_filter_enable_) {
      return false;
    }
    if (!route_lanelet_mask_ || !route_lanelet_mask_has_allowed_cells_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "route lanelet obstacle filter waiting for a valid active-route mask; passing costs through");
      return false;
    }
    if (route_lanelet_mask_max_age_s_ > 0.0 &&
      (now_time - route_lanelet_mask_receive_time_).seconds() > route_lanelet_mask_max_age_s_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "route lanelet obstacle mask is stale; passing costs through");
      return false;
    }
    if (route_lanelet_mask_->header.frame_id != output_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "route lanelet obstacle mask frame mismatch (%s != %s); passing costs through",
        route_lanelet_mask_->header.frame_id.c_str(), output_frame_id_.c_str());
      return false;
    }
    if (route_lanelet_filter_fail_open_when_robot_outside_ &&
      !route_lanelet_cost_filter::isWorldPointAllowed(
        *route_lanelet_mask_, base_in_output.point.x, base_in_output.point.y,
        route_lanelet_margin_m_, route_lanelet_allowed_max_cost_))
    {
      // HH_260720 - Preserve unfiltered obstacle protection during deliberate
      // off-route campsite and parking maneuvers.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "robot is outside the active route lanelet corridor; passing obstacle costs through");
      return false;
    }
    return true;
  }

  // Implements `mapDistanceToCost` behavior.
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

  // Implements `markDisk` behavior.
  void markDisk(
    avg_msgs::msg::AvgOccupancyGrid & grid,
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

  // Implements `clearDisk` behavior.
  void clearDisk(
    avg_msgs::msg::AvgOccupancyGrid & grid,
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

  double ignoreBelowRangeForIndex(const std::size_t idx) const
  {
    if (idx < ignore_below_ranges_m_.size() && ignore_below_ranges_m_[idx] >= 0.0) {
      return ignore_below_ranges_m_[idx];
    }
    return ignore_below_range_m_;
  }

  // Implements `transformHitToOutput` behavior.
  bool transformHitToOutput(
    const avg_msgs::msg::AvgRange & msg,
    const double ignore_below_range_m,
    geometry_msgs::msg::PointStamped & hit_output)
  {
    if (!std::isfinite(msg.range)) {
      return false;
    }
    if (ignore_below_range_m > 0.0 && msg.range < ignore_below_range_m) {
      return false;
    }
    if (msg.range < msg.min_range || msg.range > msg.max_range) {
      return false;
    }
    if (msg.header.frame_id.empty()) {
      return false;
    }

    geometry_msgs::msg::PointStamped hit_sensor;
    // HH_260720 - Convert the generated range header only for the tf2 ROS point.
    hit_sensor.header = avg_msgs::conversions::toRos(msg.header);
    if (hit_sensor.header.stamp.sec == 0 && hit_sensor.header.stamp.nanosec == 0) {
      hit_sensor.header.stamp = now();
    }
    hit_sensor.point.x = msg.range;
    hit_sensor.point.y = 0.0;
    hit_sensor.point.z = 0.0;

    try {
      hit_output = tf_buffer_->transform(
        hit_sensor, output_frame_id_, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException &) {
      // HH_260315-00:00 Fallback to latest TF to avoid transient "future
      // extrapolation" drops that make marker/grid updates look unstable.
      hit_sensor.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      try {
        hit_output = tf_buffer_->transform(
          hit_sensor, output_frame_id_, tf2::durationFromSec(0.05));
        return true;
      } catch (const tf2::TransformException & ex_latest) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "radar_cost_grid TF transform failed (%s -> %s): %s",
          hit_sensor.header.frame_id.c_str(), output_frame_id_.c_str(), ex_latest.what());
        return false;
      }
    }
  }

  // Publishes `Grid` output.
  void publishGrid()
  {
    geometry_msgs::msg::PointStamped base_origin;
    // HH_260315-00:00 Anchor rolling grid with latest available TF.
    base_origin.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    base_origin.header.frame_id = base_frame_id_;
    base_origin.point.x = 0.0;
    base_origin.point.y = 0.0;
    base_origin.point.z = 0.0;

    geometry_msgs::msg::PointStamped base_in_output;
    try {
      base_in_output = tf_buffer_->transform(
        base_origin, output_frame_id_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "radar_cost_grid failed to locate %s in %s: %s",
        base_frame_id_.c_str(), output_frame_id_.c_str(), ex.what());
      return;
    }

    const double grid_origin_x = base_in_output.point.x + origin_x_;
    const double grid_origin_y = base_in_output.point.y + origin_y_;

    avg_msgs::msg::AvgOccupancyGrid grid;
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

    // HH_260703 - Clear the ego footprint before marking live radar hits so
    // valid near-field side/rear detections can override the self-clear disk.
    if (ego_clear_radius_m_ > 0.0) {
      clearDisk(
        grid, grid_origin_x, grid_origin_y,
        base_in_output.point.x, base_in_output.point.y);
    }

    const auto now_time = now();
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const auto & sample = samples_[i];
      if (!sample.valid) {
        continue;
      }
      if ((now_time - sample.recv_time).seconds() > max_message_age_s_) {
        continue;
      }

      geometry_msgs::msg::PointStamped hit_output;
      if (!transformHitToOutput(sample.msg, ignoreBelowRangeForIndex(i), hit_output)) {
        continue;
      }
      // HH_260630: SEN0592 risk should follow sensor-relative range, not
      // robot-base distance. Front/side/rear mounts are offset from base_link,
      // so base-distance scaling made close hand/obstacle hits fall below the
      // cmd_vel gate threshold.
      const int value = mapDistanceToCost(sample.msg.range);
      markDisk(
        grid, grid_origin_x, grid_origin_y, hit_output.point.x, hit_output.point.y, value);
    }

    // HH_260720 - Clip fully-painted radar disks at the route lanelet margin,
    // preventing a hit in an adjacent lane from entering the driven corridor.
    if (shouldApplyRouteLaneletFilter(base_in_output, now_time)) {
      const auto removed = route_lanelet_cost_filter::removeCostsOutsideRouteLanelets(
        grid, *route_lanelet_mask_, route_lanelet_margin_m_,
        route_lanelet_allowed_max_cost_, free_value_);
      if (removed > 0U) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "removed %zu radar cost cells outside active route lanelets + %.2f m margin",
          removed, route_lanelet_margin_m_);
      }
    }

    pub_grid_->publish(grid);
    publishAvgRadar(grid);
  }

  // Implements `assignRangeByTopic` behavior.
  void assignRangeByTopic(AvgSensingRadar & avg_msg, std::size_t idx, const avg_msgs::msg::AvgRange & msg)
  {
    if (idx >= input_topics_.size()) {
      return;
    }
    const auto & topic = input_topics_[idx];
    // HH_260623 - Publish front1/front2 separately; merged front output was removed.
    if (topic.find("front1") != std::string::npos) {
      avg_msg.front1 = msg;
    } else if (topic.find("front2") != std::string::npos) {
      avg_msg.front2 = msg;
    } else if (topic.find("right1") != std::string::npos) {
      avg_msg.right1 = msg;
    } else if (topic.find("right2") != std::string::npos) {
      avg_msg.right2 = msg;
    } else if (topic.find("left1") != std::string::npos) {
      avg_msg.left1 = msg;
    } else if (topic.find("left2") != std::string::npos) {
      avg_msg.left2 = msg;
    } else if (topic.find("rear") != std::string::npos) {
      avg_msg.rear = msg;
    }
  }

  // Publishes `AvgRadar` output.
  void publishAvgRadar(const avg_msgs::msg::AvgOccupancyGrid & grid)
  {
    if (!publish_radar_status_ || !avg_radar_pub_) {
      return;
    }
    AvgSensingRadar avg_msg;
    // HH_260720 - Bundle the already-generated CAMROD grid without reconversion.
    avg_msg.near_cost_grid = grid;
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      if (!samples_[i].valid) {
        continue;
      }
      assignRangeByTopic(avg_msg, i, samples_[i].msg);
    }
    avg_radar_pub_->publish(avg_msg);
  }

  std::string output_topic_;
  std::string radar_status_topic_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  double resolution_{0.10};
  int width_{120};
  int height_{120};
  double origin_x_{-6.0};
  double origin_y_{-6.0};
  int free_value_{0};
  int unknown_value_{0};
  int min_cost_{85};
  int max_cost_{100};
  double cost_range_min_m_{0.3};
  double ignore_below_range_m_{0.0};
  std::vector<double> ignore_below_ranges_m_;
  double cost_range_max_m_{2.0};
  double obstacle_radius_m_{0.30};
  double ego_clear_radius_m_{0.50};
  double max_message_age_s_{0.35};
  double publish_rate_hz_{10.0};
  bool publish_radar_status_{false};
  bool route_lanelet_filter_enable_{true};
  std::string route_lanelet_mask_topic_{"/map/cost_grid/route_lanelet_mask"};
  double route_lanelet_margin_m_{0.35};
  int route_lanelet_allowed_max_cost_{50};
  double route_lanelet_mask_max_age_s_{2.5};
  bool route_lanelet_filter_fail_open_when_robot_outside_{true};
  bool route_lanelet_mask_has_allowed_cells_{false};
  std::vector<std::string> input_topics_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Publisher<AvgSensingRadar>::SharedPtr avg_radar_pub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr route_lanelet_mask_sub_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::AvgRange>::SharedPtr> subs_;
  std::vector<RangeSample> samples_;
  avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr route_lanelet_mask_;
  rclcpp::Time route_lanelet_mask_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::sensing::RadarCostGridNode>());
  rclcpp::shutdown();
  return 0;
}
