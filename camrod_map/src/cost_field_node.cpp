#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <avg_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/marker_array.hpp>

#include <avg_msgs/msg/avg_map_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

#include "camrod_map/custom_regulatory_elements.hpp"
#include <avg_msgs/msg/marker.hpp>

namespace
{
// Builds avg_msgs::Point helper objects for marker generation.
avg_msgs::msg::Point makePoint(double x, double y, double z)
{
  avg_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// Approximates local path curvature magnitude from three consecutive points.
double curvature3p(
  const lanelet::ConstPoint3d & p0,
  const lanelet::ConstPoint3d & p1,
  const lanelet::ConstPoint3d & p2)
{
  const double x1 = p1.x() - p0.x();
  const double y1 = p1.y() - p0.y();
  const double x2 = p2.x() - p1.x();
  const double y2 = p2.y() - p1.y();
  const double cross = std::abs(x1 * y2 - y1 * x2);
  const double norm1 = std::hypot(x1, y1);
  const double norm2 = std::hypot(x2, y2);
  const double denom = std::pow(norm1 * norm2, 2.0 / 3.0);
  if (denom < 1e-6) {
    return 0.0;
  }
  return cross / denom;
}

// Maps normalized cost [0,1] into an RGB color for RViz line markers.
std::array<float, 4> colorFromCost(double cost)
{
  const double c = std::clamp(cost, 0.0, 1.0);
  float r = static_cast<float>(c);
  float g = static_cast<float>(1.0 - c);
  float b = 0.2f;
  float a = 0.8f;
  return {r, g, b, a};
}

// Normalizes projector type strings for robust parameter parsing.
std::string normalizeProjectorType(const std::string & value)
{
  std::string out = value;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}
}  // namespace

namespace camrod_map
{

struct CostWeights
{
  double distance{1.0};
  double curvature{1.0};
  double lane_preference{0.0};
};

class CostFieldNode : public rclcpp::Node
{
public:
  // Declares parameters, loads map geometry, and starts periodic marker publishing.
  CostFieldNode()
  : Node("cost_field")
  {
    config_.map_path = declare_parameter<std::string>("map_path", "");
    config_.offset_lat = declare_parameter<double>("offset_lat", 0.0);
    config_.offset_lon = declare_parameter<double>("offset_lon", 0.0);
    config_.offset_alt = declare_parameter<double>("offset_alt", 0.0);
    config_.map_frame_id = declare_parameter<std::string>("map_frame_id", "map");
    config_.projector_type = declare_parameter<std::string>("projector_type", "local_cartesian");
    config_.mgrs_grid = declare_parameter<std::string>("mgrs_grid", "");
    config_.max_draw_distance = declare_parameter<double>("max_draw_distance", 0.0);
    config_.percentile_clip = declare_parameter<double>("percentile_clip", 0.95);
    config_.output_topic = declare_parameter<std::string>(
      "output_topic", "/map/cost_grid/lanelet_field_markers");
    publish_map_status_ = declare_parameter<bool>("publish_map_status", false);
    map_status_topic_ = declare_parameter<std::string>("map_status_topic", "/map/status");
    weights_.distance = declare_parameter<double>("weights.distance", 1.0);
    weights_.curvature = declare_parameter<double>("weights.curvature", 0.5);
    weights_.lane_preference = declare_parameter<double>("weights.lane_preference", 0.0);

    if (config_.map_path.empty()) {
      RCLCPP_ERROR(get_logger(), "map_path is empty.");
      rclcpp::shutdown();
      return;
    }

    if (!loadMap()) {
      rclcpp::shutdown();
      return;
    }

    pub_markers_ = create_publisher<avg_msgs::msg::MarkerArray>(
      config_.output_topic, rclcpp::QoS(1).transient_local());
    if (publish_map_status_) {
      avg_map_pub_ = create_publisher<avg_msgs::msg::AvgMapMsgs>(
        map_status_topic_, rclcpp::QoS(10));
    }

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1s, std::bind(&CostFieldNode::publishMarkers, this));

    RCLCPP_INFO(get_logger(), "Cost field ready. map=%s", config_.map_path.c_str());
  }

private:
  // Builds lanelet-segment cost markers and publishes the latest marker array.
  void publishMarkers()
  {
    avg_msgs::msg::MarkerArray arr;
    arr.markers.reserve(loadedPointCount());

    const rclcpp::Time stamp = this->now();
    int32_t id = 0;

    struct Seg
    {
      avg_msgs::msg::Point p0;
      avg_msgs::msg::Point p1;
      double cost;
    };

    std::vector<Seg> segments;
    segments.reserve(loadedPointCount());

    double max_cost = 1e-9;
    double min_cost = std::numeric_limits<double>::max();
    double sum_cost = 0.0;
    size_t cnt_cost = 0;
    std::vector<double> cost_samples;

    for (const auto & ll : map_->laneletLayer) {
      const auto & cl = ll.centerline();
      if (cl.size() < 2) {
        continue;
      }
      for (size_t i = 0; i + 1 < cl.size(); ++i) {
        const auto & p0 = cl[i];
        const auto & p1 = cl[i + 1];
        const double dist = std::hypot(p1.x() - p0.x(), p1.y() - p0.y());
        const double curv = (i + 2 < cl.size()) ? curvature3p(p0, p1, cl[i + 2]) : 0.0;
        const double cost = weights_.distance * dist + weights_.curvature * curv;

        max_cost = std::max(max_cost, cost);
        min_cost = std::min(min_cost, cost);
        sum_cost += cost;
        ++cnt_cost;
        cost_samples.push_back(cost);
        segments.push_back({makePoint(p0.x(), p0.y(), 0.0), makePoint(p1.x(), p1.y(), 0.0), cost});
      }
    }

    double clip_cost = max_cost;
    if (!cost_samples.empty()) {
      const double pct = std::clamp(config_.percentile_clip, 0.0, 1.0);
      if (pct > 0.0 && pct < 1.0) {
        const size_t idx = static_cast<size_t>(pct * static_cast<double>(cost_samples.size() - 1));
        std::nth_element(cost_samples.begin(), cost_samples.begin() + idx, cost_samples.end());
        clip_cost = std::max(cost_samples[idx], 1e-6);
      }
    }

    for (auto & seg : segments) {
      const double norm = std::clamp(seg.cost / clip_cost, 0.0, 1.0);
      auto color = colorFromCost(norm);

      avg_msgs::msg::Marker line;
      line.header.frame_id = config_.map_frame_id;
      line.header.stamp = stamp;
      line.ns = "cost_field";
      line.id = id++;
      line.type = avg_msgs::msg::Marker::LINE_LIST;
      line.action = avg_msgs::msg::Marker::ADD;
      line.scale.x = 0.25;
      line.color.r = color[0];
      line.color.g = color[1];
      line.color.b = color[2];
      line.color.a = color[3];

      if (config_.max_draw_distance > 1e-3) {
        const double d0 = std::hypot(seg.p0.x, seg.p0.y);
        const double d1 = std::hypot(seg.p1.x, seg.p1.y);
        if (d0 > config_.max_draw_distance && d1 > config_.max_draw_distance) {
          continue;
        }
      }

      line.points.push_back(seg.p0);
      line.points.push_back(seg.p1);
      arr.markers.emplace_back(std::move(line));
    }

    if (cnt_cost > 0) {
      const double avg = sum_cost / static_cast<double>(cnt_cost);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CostField stats min=%.3f max=%.3f avg=%.3f (clip=%.3f, norm by clip)",
        min_cost, max_cost, avg, clip_cost);
    }

    pub_markers_->publish(arr);
    publishAvgMapMessage(arr, stamp);
  }

  // Publishes unified avg_msgs map status payload for cost-field outputs.
  void publishAvgMapMessage(const avg_msgs::msg::MarkerArray & markers, const rclcpp::Time & stamp)
  {
    if (!publish_map_status_ || !avg_map_pub_) {
      return;
    }
    avg_msgs::msg::AvgMapMsgs msg;
    msg.stamp = stamp;
    msg.state.stamp = stamp;
    msg.state.module_name = "map";
    msg.state.level = avg_msgs::msg::ModuleState::OK;
    msg.state.message = "lanelet field markers published";
    msg.lanelet_markers = markers;
    avg_map_pub_->publish(msg);
  }

  // Returns total centerline segment count used for reserve sizing.
  size_t loadedPointCount() const
  {
    size_t sum = 0;
    for (const auto & ll : map_->laneletLayer) {
      if (ll.centerline().size() > 1) {
        sum += ll.centerline().size() - 1;
      }
    }
    return sum;
  }

  // Loads lanelet map data with the selected projector configuration.
  bool loadMap()
  {
    lanelet::GPSPoint gps{config_.offset_lat, config_.offset_lon, config_.offset_alt};
    lanelet::Origin origin(gps);
    std::unique_ptr<lanelet::Projector> projector;

    const auto proj_type = normalizeProjectorType(config_.projector_type);
    if (proj_type == "utm" || proj_type == "transversemercator" || proj_type == "mgrs") {
      projector = std::make_unique<lanelet::projection::UtmProjector>(origin, false);
    } else {
      projector = std::make_unique<lanelet::projection::LocalCartesianProjector>(origin);
    }

    lanelet::ErrorMessages errs;
    map_ = lanelet::load(config_.map_path, *projector, &errs);
    for (const auto & e : errs) {
      RCLCPP_WARN(get_logger(), "lanelet load warning: %s", e.c_str());
    }
    if (!map_) {
      RCLCPP_FATAL(get_logger(), "Failed to load map: %s", config_.map_path.c_str());
      return false;
    }
    return true;
  }

  struct Config
  {
    std::string map_path;
    double offset_lat{0.0};
    double offset_lon{0.0};
    double offset_alt{0.0};
    std::string map_frame_id{"map"};
    std::string projector_type{"local_cartesian"};
    std::string mgrs_grid;
    double max_draw_distance{0.0};
    double percentile_clip{0.95};
    std::string output_topic{"/map/cost_grid/lanelet_field_markers"};
  } config_;

  CostWeights weights_;
  bool publish_map_status_{false};
  std::string map_status_topic_{"/map/status"};
  lanelet::LaneletMapPtr map_;
  rclcpp::Publisher<avg_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<avg_msgs::msg::AvgMapMsgs>::SharedPtr avg_map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_map

// Entrypoint that spins the lanelet cost-field visualizer node.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_map::CostFieldNode>());
  rclcpp::shutdown();
  return 0;
}
