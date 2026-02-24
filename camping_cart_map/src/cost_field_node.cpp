#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

#include "camping_cart_map/custom_regulatory_elements.hpp"  // HH_251231 ensure speed_bump rule is registered
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

double curvature3p(const lanelet::ConstPoint3d & p0, const lanelet::ConstPoint3d & p1, const lanelet::ConstPoint3d & p2)
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

std::array<float, 4> colorFromCost(double cost)
{
  // HH_251230: lower cost -> green, higher cost -> red (gradient)
  const double c = std::clamp(cost, 0.0, 1.0);
  float r = static_cast<float>(c);
  float g = static_cast<float>(1.0 - c);
  float b = 0.2f;
  float a = 0.8f;
  return {r, g, b, a};
}
}  // namespace

namespace camping_cart::localization
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
// HH_260112 Use short node name; namespace applies the module prefix.
CostFieldNode() : Node("cost_field")
  {
    config_.map_path = declare_parameter<std::string>("map_path", "");
    config_.offset_lat = declare_parameter<double>("offset_lat", 0.0);
    config_.offset_lon = declare_parameter<double>("offset_lon", 0.0);
    config_.offset_alt = declare_parameter<double>("offset_alt", 0.0);
    config_.map_frame_id = declare_parameter<std::string>("map_frame_id", "map");
    config_.projector_type = declare_parameter<std::string>("projector_type", "LocalCartesianUTM");
    config_.mgrs_grid = declare_parameter<std::string>("mgrs_grid", "");
    config_.max_draw_distance = declare_parameter<double>("max_draw_distance", 0.0);  // 0 -> unlimited
    config_.percentile_clip = declare_parameter<double>("percentile_clip", 0.95);     // 0.0~1.0
    config_.output_topic = declare_parameter<std::string>(
      "output_topic", "/map/cost_grid/lanelet_field_markers");
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

    // HH_260109 Visualizer topic prefix for cost field markers.
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.output_topic, rclcpp::QoS(1).transient_local());

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1s, std::bind(&CostFieldNode::publishMarkers, this));

    RCLCPP_INFO(get_logger(), "Cost field ready. map=%s", config_.map_path.c_str());
  }

private:
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.reserve(loadedPointCount());

    const rclcpp::Time stamp = this->now();
    int32_t id = 0;

    // HH_251230: first collect all segment costs to normalize (avoid all-red near vehicle)
    struct Seg { geometry_msgs::msg::Point p0; geometry_msgs::msg::Point p1; double cost; };
    std::vector<Seg> segments;
    segments.reserve(loadedPointCount());

    double max_cost = 1e-9;
    double min_cost = std::numeric_limits<double>::max();
    double sum_cost = 0.0;
    size_t cnt_cost = 0;
    std::vector<double> cost_samples;
    for (const auto & ll : map_->laneletLayer) {
      const auto & cl = ll.centerline();
      if (cl.size() < 2) continue;
      for (size_t i = 0; i + 1 < cl.size(); ++i) {
        const auto & p0 = cl[i];
        const auto & p1 = cl[i + 1];
        // HH_251230: ignore Z so height does not affect cost
        const double dist = std::hypot(p1.x() - p0.x(), p1.y() - p0.y());
        const double curv = (i + 2 < cl.size()) ? curvature3p(p0, p1, cl[i + 2]) : 0.0;
        double cost = weights_.distance * dist + weights_.curvature * curv;
        max_cost = std::max(max_cost, cost);
        min_cost = std::min(min_cost, cost);
        sum_cost += cost;
        ++cnt_cost;
        cost_samples.push_back(cost);
        segments.push_back({makePoint(p0.x(), p0.y(), 0.0), makePoint(p1.x(), p1.y(), 0.0), cost});
      }
    }

    // HH_251231: optional percentile clipping to tame extremes
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
      const double norm = std::clamp(seg.cost / clip_cost, 0.0, 1.0);  // 0~1
      auto color = colorFromCost(norm);

      visualization_msgs::msg::Marker line;
      line.header.frame_id = config_.map_frame_id;
      line.header.stamp = stamp;
      line.ns = "cost_field";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.25;
      line.color.r = color[0];
      line.color.g = color[1];
      line.color.b = color[2];
      line.color.a = color[3];
      // HH_251231: optional draw radius limit (0 => no limit)
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
  }

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

  bool loadMap()
  {
    lanelet::GPSPoint gps{config_.offset_lat, config_.offset_lon, config_.offset_alt};
    lanelet::Origin origin(gps);
    std::unique_ptr<lanelet::Projector> projector;

    const auto proj_type = config_.projector_type;
    if (proj_type == "UTM" || proj_type == "LocalCartesianUTM" || proj_type == "TransverseMercator" || proj_type == "MGRS") {
      projector = std::make_unique<lanelet::projection::UtmProjector>(origin, false);
    } else {  // default LocalCartesian
      projector = std::make_unique<lanelet::projection::LocalCartesianProjector>(origin);
    }

    lanelet::ErrorMessages errs;
    map_ = lanelet::load(config_.map_path, *projector, &errs);
    if (!errs.empty()) {
      for (const auto & e : errs) {
        RCLCPP_WARN(get_logger(), "lanelet load warning: %s", e.c_str());
      }
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
    std::string projector_type{"LocalCartesianUTM"};
    std::string mgrs_grid;
    double max_draw_distance{0.0};
    double percentile_clip{0.95};
    std::string output_topic{"/map/cost_grid/lanelet_field_markers"};
  } config_;

  CostWeights weights_;
  lanelet::LaneletMapPtr map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camping_cart::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::localization::CostFieldNode>());
  rclcpp::shutdown();
  return 0;
}
