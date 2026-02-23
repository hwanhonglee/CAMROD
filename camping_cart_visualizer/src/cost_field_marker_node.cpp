// HH_260109 Cost field marker publisher (visualizer package).
// Subscribes to a nav_msgs/OccupancyGrid (e.g., /map/lanelet_cost_grid) and publishes
// a MarkerArray that visualizes cell costs with a red→yellow→green gradient.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
      "marker_topic", "/visualizer/map/inflation_cost_grid_markers");
    marker_scale_ = declare_parameter<double>("marker_scale", 0.2);  // HH_260101 configurable size
    min_value_ = declare_parameter<int>("min_value", 0);
    max_value_ = declare_parameter<int>("max_value", 100);
    alpha_ = declare_parameter<double>("alpha", 0.35);  // HH_260102 softer opacity
    z_offset_ = declare_parameter<double>("z_offset", 0.05);  // HH_260101 lift markers above map
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&CostFieldMarkerNode::onParamChange, this, std::placeholders::_1));

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, rclcpp::QoS(1),
      std::bind(&CostFieldMarkerNode::onGrid, this, std::placeholders::_1));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, rclcpp::QoS(1));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(), "cost_field_marker_node listening on %s, publishing %s",
      grid_topic_.c_str(), marker_topic_.c_str());

    // HH_260103 periodic republish for RViz toggle refresh
    republish_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CostFieldMarkerNode::onRepublishTimer, this));
  }

private:
  visualization_msgs::msg::Marker initMarker(const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "inflation_cost_grid";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
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
      c.r = c.g = c.b = 0.2f;
      return c;
    }
    const float denom = static_cast<float>(std::max(1, max_value_ - min_value_));
    const float norm = std::clamp(static_cast<float>(v - min_value_) / denom, 0.0f, 1.0f);
    // HH_260102 gradient: low cost = teal, high cost = muted red
    const float low_r = 0.2f, low_g = 0.8f, low_b = 0.8f;
    const float high_r = 0.8f, high_g = 0.2f, high_b = 0.2f;
    c.r = high_r * norm + low_r * (1.0f - norm);
    c.g = high_g * norm + low_g * (1.0f - norm);
    c.b = high_b * norm + low_b * (1.0f - norm);
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
      }
    }
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    r.reason = "updated";
    return r;
  }

  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    auto marker = initMarker(msg->header);
    marker.scale.x = msg->info.resolution;
    marker.scale.y = msg->info.resolution;
    marker.scale.z = msg->info.resolution * 0.5;

    const auto & info = msg->info;
    const size_t width = info.width;
    const size_t height = info.height;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const double res = info.resolution;

    marker.points.reserve(msg->data.size());
    marker.colors.reserve(msg->data.size());

    for (size_t y = 0; y < height; ++y) {
      for (size_t x = 0; x < width; ++x) {
        const size_t idx = y * width + x;
        const int8_t v = msg->data[idx];
        if (v < 0) {
          continue;  // skip unknown to reduce clutter
        }
        geometry_msgs::msg::Point p;
        p.x = origin_x + (static_cast<double>(x) + 0.5) * res;
        p.y = origin_y + (static_cast<double>(y) + 0.5) * res;
        p.z = info.origin.position.z + z_offset_;
        marker.points.push_back(p);
        marker.colors.push_back(colorFromValue(v));
      }
    }

    arr.markers.push_back(marker);
    last_markers_ = arr;
    marker_pub_->publish(arr);
  }

  void onRepublishTimer()
  {
    if (!last_markers_.markers.empty()) {
      for (auto & m : last_markers_.markers) {
        m.header.stamp = now();
      }
      marker_pub_->publish(last_markers_);
    }
  }

  std::string grid_topic_;
  std::string marker_topic_;
  double marker_scale_{0.2};
  int min_value_{0};
  int max_value_{100};
  double alpha_{0.35};
  double z_offset_{0.05};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray last_markers_;
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
