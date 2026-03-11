// 2026-02-24: Cost field marker publisher under map package.
// Subscribes to a nav_msgs/OccupancyGrid (e.g., /planning/global_costmap/costmap) and publishes
// a MarkerArray that visualizes cell costs.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <cmath>

namespace camping_cart_map
{

class CostFieldMarkerNode : public rclcpp::Node
{
public:
// HH_260112 Use short node name; namespace applies the module prefix.
CostFieldMarkerNode() : Node("cost_field_marker")
  {
    // 2026-02-02 11:10: Default to combined (inflation-applied) Nav2 costmap output.
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/planning/global_costmap/costmap");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/map/cost_grid/inflation_markers");
    marker_scale_ = declare_parameter<double>("marker_scale", 0.2);  // HH_260101 configurable size
    min_value_ = declare_parameter<int>("min_value", 0);
    max_value_ = declare_parameter<int>("max_value", 100);
    alpha_ = declare_parameter<double>("alpha", 0.35);  // HH_260102 softer opacity
    z_offset_ = declare_parameter<double>("z_offset", 0.05);  // HH_260101 lift markers above map
    cell_scale_ratio_ = declare_parameter<double>("cell_scale_ratio", 1.0);
    palette_ = declare_parameter<std::string>("palette", "safety");
    show_unknown_ = declare_parameter<bool>("show_unknown", false);
    clear_on_empty_grid_ = declare_parameter<bool>("clear_on_empty_grid", true);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.0);
    grid_qos_transient_local_ = declare_parameter<bool>("grid_qos_transient_local", false);
    min_publish_period_sec_ = declare_parameter<double>("min_publish_period_sec", 0.0);
    republish_period_sec_ = declare_parameter<double>("republish_period_sec", 0.0);
    sample_stride_ = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("sample_stride", 1)));
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&CostFieldMarkerNode::onParamChange, this, std::placeholders::_1));

    auto grid_qos = rclcpp::QoS(1).reliable();
    if (grid_qos_transient_local_) {
      grid_qos.transient_local();
    }
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, grid_qos,
      std::bind(&CostFieldMarkerNode::onGrid, this, std::placeholders::_1));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).transient_local().reliable());

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(), "cost_field_marker_node listening on %s, publishing %s",
      grid_topic_.c_str(), marker_topic_.c_str());

    // HH_260103 periodic republish for RViz toggle refresh
    updateRepublishTimer();
  }

private:
  bool canPublishNow() const
  {
    if (min_publish_period_sec_ <= 0.0 || last_publish_time_.nanoseconds() <= 0) {
      return true;
    }
    const double dt = (now() - last_publish_time_).seconds();
    return dt >= min_publish_period_sec_;
  }

  void publishFromGrid(const nav_msgs::msg::OccupancyGrid & grid_msg)
  {
    visualization_msgs::msg::MarkerArray arr;
    auto marker = initMarker(grid_msg.header);
    const double stride = static_cast<double>(sample_stride_);
    marker.scale.x = grid_msg.info.resolution * cell_scale_ratio_ * stride;
    marker.scale.y = grid_msg.info.resolution * cell_scale_ratio_ * stride;
    marker.scale.z = grid_msg.info.resolution * 0.5;

    const auto & info = grid_msg.info;
    const size_t width = info.width;
    const size_t height = info.height;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const double res = info.resolution;
    const auto & q = info.origin.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // HH_260305-00:00 Reserve by sampled-cell upper bound (not full grid size).
    // This reduces allocation pressure on large maps and improves RViz latency.
    const std::size_t sampled_w =
      (width + static_cast<std::size_t>(sample_stride_) - 1U) / static_cast<std::size_t>(sample_stride_);
    const std::size_t sampled_h =
      (height + static_cast<std::size_t>(sample_stride_) - 1U) / static_cast<std::size_t>(sample_stride_);
    const std::size_t sampled_cap = sampled_w * sampled_h;
    marker.points.reserve(sampled_cap);
    marker.colors.reserve(sampled_cap);

    for (size_t y = 0; y < height; y += static_cast<size_t>(sample_stride_)) {
      for (size_t x = 0; x < width; x += static_cast<size_t>(sample_stride_)) {
        const size_t idx = y * width + x;
        const int8_t v = grid_msg.data[idx];
        if (v < 0 && !show_unknown_) {
          continue;
        }
        geometry_msgs::msg::Point p;
        const double lx = (static_cast<double>(x) + 0.5 * stride) * res;
        const double ly = (static_cast<double>(y) + 0.5 * stride) * res;
        p.x = origin_x + cy * lx - sy * ly;
        p.y = origin_y + sy * lx + cy * ly;
        p.z = info.origin.position.z + z_offset_;
        marker.points.push_back(p);
        marker.colors.push_back(colorFromValue(v));
      }
    }

    arr.markers.push_back(marker);
    last_markers_ = arr;
    marker_pub_->publish(arr);
    last_publish_time_ = now();
    pending_grid_update_ = false;
  }

  visualization_msgs::msg::Marker initMarker(const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "inflation_cost_grid";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = marker_scale_;  // default; will be overridden by resolution
    m.scale.y = marker_scale_;
    m.scale.z = marker_scale_;
    m.lifetime = rclcpp::Duration(0, 0);
    return m;
  }

  std_msgs::msg::ColorRGBA colorFromValue(int8_t v) const
  {
    std_msgs::msg::ColorRGBA c;
    c.a = static_cast<float>(alpha_);
    if (v < 0) {  // unknown
      if (palette_ == "pastel") {
        c.r = 0.85f; c.g = 0.85f; c.b = 0.88f;
      } else {
        c.r = c.g = c.b = 0.2f;
      }
      return c;
    }
    const float denom = static_cast<float>(std::max(1, max_value_ - min_value_));
    const float norm = std::clamp(static_cast<float>(v - min_value_) / denom, 0.0f, 1.0f);
    if (palette_ == "pastel") {
      const float low_r = 0.72f, low_g = 0.86f, low_b = 0.95f;    // pastel blue
      const float high_r = 0.95f, high_g = 0.80f, high_b = 0.86f; // pastel pink
      c.r = high_r * norm + low_r * (1.0f - norm);
      c.g = high_g * norm + low_g * (1.0f - norm);
      c.b = high_b * norm + low_b * (1.0f - norm);
      return c;
    }

    // safety palette (default):
    // low  -> blue/green (drivable / safer)
    // high -> red (high risk / near obstacle)
    const float low_r = 0.20f, low_g = 0.70f, low_b = 0.95f;
    const float mid_r = 0.20f, mid_g = 0.85f, mid_b = 0.35f;
    const float high_r = 0.92f, high_g = 0.16f, high_b = 0.14f;
    if (norm < 0.5f) {
      const float t = norm / 0.5f;
      c.r = mid_r * t + low_r * (1.0f - t);
      c.g = mid_g * t + low_g * (1.0f - t);
      c.b = mid_b * t + low_b * (1.0f - t);
    } else {
      const float t = (norm - 0.5f) / 0.5f;
      c.r = high_r * t + mid_r * (1.0f - t);
      c.g = high_g * t + mid_g * (1.0f - t);
      c.b = high_b * t + mid_b * (1.0f - t);
    }
    return c;
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "marker_scale") {
        marker_scale_ = p.as_double();
      } else if (p.get_name() == "min_value") {
        min_value_ = p.as_int();
      } else if (p.get_name() == "max_value") {
        max_value_ = p.as_int();
      } else if (p.get_name() == "alpha") {
        alpha_ = p.as_double();
      } else if (p.get_name() == "z_offset") {
        z_offset_ = p.as_double();
      } else if (p.get_name() == "cell_scale_ratio") {
        cell_scale_ratio_ = std::clamp(p.as_double(), 0.1, 1.0);
      } else if (p.get_name() == "palette") {
        palette_ = p.as_string();
      } else if (p.get_name() == "show_unknown") {
        show_unknown_ = p.as_bool();
      } else if (p.get_name() == "clear_on_empty_grid") {
        clear_on_empty_grid_ = p.as_bool();
      } else if (p.get_name() == "stale_timeout_sec") {
        stale_timeout_sec_ = p.as_double();
      } else if (p.get_name() == "grid_qos_transient_local") {
        grid_qos_transient_local_ = p.as_bool();
      } else if (p.get_name() == "min_publish_period_sec") {
        min_publish_period_sec_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "republish_period_sec") {
        republish_period_sec_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "sample_stride") {
        sample_stride_ = std::max(1, static_cast<int>(p.as_int()));
      }
    }
    updateRepublishTimer();
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    r.reason = "updated";
    return r;
  }

  void updateRepublishTimer()
  {
    republish_timer_.reset();
    if (republish_period_sec_ <= 0.0) {
      return;
    }

    republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(republish_period_sec_)),
      std::bind(&CostFieldMarkerNode::onRepublishTimer, this));
  }

  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_grid_ = msg;
    pending_grid_update_ = true;
    last_grid_rx_ = now();
    if (msg->data.empty()) {
      if (clear_on_empty_grid_) {
        publishDeleteAll(msg->header);
      }
      latest_grid_.reset();
      pending_grid_update_ = false;
      return;
    }
    if (!canPublishNow()) {
      return;
    }
    publishFromGrid(*msg);
  }

  void onRepublishTimer()
  {
    if (stale_timeout_sec_ > 0.0 && last_grid_rx_.nanoseconds() > 0) {
      const double dt = (now() - last_grid_rx_).seconds();
      if (dt > stale_timeout_sec_ && !last_markers_.markers.empty()) {
        publishDeleteAll(last_markers_.markers.front().header);
        latest_grid_.reset();
        pending_grid_update_ = false;
        return;
      }
    }

    // HH_260307-00:00 Prioritize pending fresh grid conversion over old marker heartbeat.
    // Without this, a throttled onGrid() could leave RViz stuck on stale markers.
    if (pending_grid_update_ && latest_grid_ && !latest_grid_->data.empty() && canPublishNow()) {
      publishFromGrid(*latest_grid_);
      return;
    }

    if (!last_markers_.markers.empty()) {
      for (auto & m : last_markers_.markers) {
        m.header.stamp = now();
      }
      marker_pub_->publish(last_markers_);
      last_publish_time_ = now();
      return;
    }

    // HH_260305-00:00 Fallback: generate first marker set only when cache is empty.
    // Periodic timer should primarily republish cached markers to avoid expensive full-grid scans.
    if (latest_grid_ && !latest_grid_->data.empty() && canPublishNow()) {
      publishFromGrid(*latest_grid_);
    }
  }

  void publishDeleteAll(const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker del;
    del.header = header;
    del.ns = "inflation_cost_grid";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);
    marker_pub_->publish(arr);
    last_markers_.markers.clear();
  }

  std::string grid_topic_;
  std::string marker_topic_;
  double marker_scale_{0.2};
  int min_value_{0};
  int max_value_{100};
  double alpha_{0.35};
  double z_offset_{0.05};
  double cell_scale_ratio_{1.0};
  std::string palette_{"pastel"};
  bool show_unknown_{false};
  bool clear_on_empty_grid_{true};
  double stale_timeout_sec_{0.0};
  bool grid_qos_transient_local_{false};
  double min_publish_period_sec_{0.0};
  double republish_period_sec_{0.0};
  int sample_stride_{1};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray last_markers_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_grid_;
  bool pending_grid_update_{false};
  rclcpp::Time last_grid_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr republish_timer_;
};

}  // namespace camping_cart_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart_map::CostFieldMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
