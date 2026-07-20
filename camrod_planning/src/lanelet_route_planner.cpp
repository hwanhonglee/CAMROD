#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
// HH_260720 - Publish typed CAMROD route metadata instead of generic ROS arrays.
#include "avg_msgs/msg/route_lanelet_ids.hpp"
#include "avg_msgs/msg/route_turn_segment.hpp"
#include "avg_msgs/msg/route_turn_segment_array.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_io/Projection.h"
#include "lanelet2_projection/LocalCartesian.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "camrod_map/custom_regulatory_elements.hpp"

namespace camrod_planning
{
namespace
{
struct ProjectedPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double heading{0.0};
  double arc_length{0.0};
  double distance_sq{std::numeric_limits<double>::max()};
  bool valid{false};
};

struct LaneletMatch
{
  lanelet::ConstLanelet lanelet;
  ProjectedPoint projection;
  bool inside{false};
  bool valid{false};
};

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double half_yaw = 0.5 * yaw;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(half_yaw);
  quaternion.w = std::cos(half_yaw);
  return quaternion;
}

bool pointInPolygon2D(
  const std::vector<std::pair<double, double>> & polygon, const double x,
  const double y)
{
  if (polygon.size() < 3U) {
    return false;
  }

  bool inside = false;
  for (std::size_t point_index = 0U, previous_index = polygon.size() - 1U;
    point_index < polygon.size(); previous_index = point_index++)
  {
    const double current_x = polygon[point_index].first;
    const double current_y = polygon[point_index].second;
    const double previous_x = polygon[previous_index].first;
    const double previous_y = polygon[previous_index].second;
    const bool crosses_ray = ((current_y > y) != (previous_y > y)) &&
      (x < (previous_x - current_x) * (y - current_y) /
      ((previous_y - current_y) + 1.0e-9) + current_x);
    if (crosses_ray) {
      inside = !inside;
    }
  }
  return inside;
}
}  // namespace

class LaneletRoutePlanner : public nav2_core::GlobalPlanner
{
public:
  ~LaneletRoutePlanner() override
  {
    joinInitializationThread();
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    (void)tf;
    (void)costmap_ros;

    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error("LaneletRoutePlanner failed to lock lifecycle node");
    }

    plugin_name_ = std::move(name);
    declareIfMissing(plugin_name_ + ".map_path", std::string(""));
    declareIfMissing(plugin_name_ + ".offset_lat", 0.0);
    declareIfMissing(plugin_name_ + ".offset_lon", 0.0);
    declareIfMissing(plugin_name_ + ".offset_alt", 0.0);
    declareIfMissing(plugin_name_ + ".frame_id", std::string("map"));
    declareIfMissing(plugin_name_ + ".routing_location", std::string("de"));
    declareIfMissing(plugin_name_ + ".routing_participant", std::string("vehicle:car"));
    declareIfMissing(plugin_name_ + ".allow_lane_changes", true);
    declareIfMissing(plugin_name_ + ".max_snap_distance_m", 8.0);
    declareIfMissing(plugin_name_ + ".interpolation_resolution_m", 0.20);
    declareIfMissing(plugin_name_ + ".same_lane_forward_epsilon_m", 0.30);
    declareIfMissing(plugin_name_ + ".flatten_path_z", true);
    declareIfMissing(
      plugin_name_ + ".route_lanelet_ids_topic",
      std::string("/planning/route_lanelet_ids"));
    declareIfMissing(plugin_name_ + ".async_initialization", true);
    declareIfMissing(plugin_name_ + ".async_initialization_plan_wait_timeout_s", 60.0);
    declareIfMissing(
      plugin_name_ + ".route_turn_segments_topic", std::string("/planning/route_turn_segments"));

    node_->get_parameter(plugin_name_ + ".map_path", map_path_);
    node_->get_parameter(plugin_name_ + ".offset_lat", offset_lat_);
    node_->get_parameter(plugin_name_ + ".offset_lon", offset_lon_);
    node_->get_parameter(plugin_name_ + ".offset_alt", offset_alt_);
    node_->get_parameter(plugin_name_ + ".frame_id", frame_id_);
    node_->get_parameter(plugin_name_ + ".routing_location", routing_location_);
    node_->get_parameter(plugin_name_ + ".routing_participant", routing_participant_);
    node_->get_parameter(plugin_name_ + ".allow_lane_changes", allow_lane_changes_);
    node_->get_parameter(plugin_name_ + ".max_snap_distance_m", max_snap_distance_m_);
    node_->get_parameter(plugin_name_ + ".interpolation_resolution_m", interpolation_resolution_m_);
    node_->get_parameter(
      plugin_name_ + ".same_lane_forward_epsilon_m",
      same_lane_forward_epsilon_m_);
    node_->get_parameter(plugin_name_ + ".flatten_path_z", flatten_path_z_);
    node_->get_parameter(plugin_name_ + ".route_lanelet_ids_topic", route_lanelet_ids_topic_);
    node_->get_parameter(plugin_name_ + ".async_initialization", async_initialization_);
    node_->get_parameter(
      plugin_name_ + ".async_initialization_plan_wait_timeout_s",
      async_initialization_plan_wait_timeout_s_);
    node_->get_parameter(plugin_name_ + ".route_turn_segments_topic", route_turn_segments_topic_);

    interpolation_resolution_m_ = std::max(0.05, interpolation_resolution_m_);
    max_snap_distance_m_ = std::max(0.1, max_snap_distance_m_);

    // HH_260619 - Publish exact routing lanelet IDs so route-aware cost grids do
    // not infer ambiguous lanelets from overlapping merge polygons.
    route_lanelet_ids_pub_ = node_->create_publisher<avg_msgs::msg::RouteLaneletIds>(
      route_lanelet_ids_topic_, rclcpp::QoS(1).reliable().transient_local());
    // HH_260708 - Publish turn_direction tag windows as route arc-length ranges so
    // the exterior light controller can pre-signal turns without loading the map.
    route_turn_segments_pub_ = node_->create_publisher<avg_msgs::msg::RouteTurnSegmentArray>(
      route_turn_segments_topic_, rclcpp::QoS(1).reliable().transient_local());

    startInitialization();
  }

  void cleanup() override
  {
    joinInitializationThread();
    route_lanelet_ids_pub_.reset();
    route_turn_segments_pub_.reset();
    traffic_rules_.reset();
    routing_graph_.reset();
    map_.reset();
    std::lock_guard<std::mutex> lock(initialization_mutex_);
    initialization_started_ = false;
    initialization_complete_ = false;
    initialization_success_ = false;
    initialization_error_.clear();
  }

  void activate() override
  {
    if (route_lanelet_ids_pub_) {
      route_lanelet_ids_pub_->on_activate();
    }
    if (route_turn_segments_pub_) {
      route_turn_segments_pub_->on_activate();
    }
  }

  void deactivate() override
  {
    if (route_lanelet_ids_pub_) {
      route_lanelet_ids_pub_->on_deactivate();
    }
    if (route_turn_segments_pub_) {
      route_turn_segments_pub_->on_deactivate();
    }
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    if (!waitForInitialization()) {
      throw nav2_core::PlannerException(
              "LaneletRoutePlanner map/routing graph initialization is not ready");
    }

    const auto plan_t0 = std::chrono::steady_clock::now();
    const LaneletMatch start_match = findBestLanelet(
      start.pose.position.x, start.pose.position.y);
    const LaneletMatch goal_match = findBestLanelet(
      goal.pose.position.x, goal.pose.position.y);
    const auto snap_t1 = std::chrono::steady_clock::now();

    if (!start_match.valid) {
      throw nav2_core::PlannerException("LaneletRoutePlanner could not snap start to lanelet");
    }
    if (!goal_match.valid) {
      throw nav2_core::PlannerException("LaneletRoutePlanner could not snap goal to lanelet");
    }

    const auto route_lanelets = findRoute(start_match, goal_match);
    const auto route_t1 = std::chrono::steady_clock::now();
    if (route_lanelets.empty()) {
      throw nav2_core::PlannerException("LaneletRoutePlanner could not find lanelet route");
    }
    publishRouteLaneletIds(route_lanelets);

    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = frame_id_;

    // HH_260619 - Route geometry comes from Lanelet centerlines, not costmap
    // free-space search. This keeps global path fixed on the legal lane route;
    // local/controller layers remain responsible for obstacle response.
    // HH_260708 - Accumulate per-lanelet arc-length windows while building the
    // path so turn_direction tags publish as [S0,S1] ranges on the same route
    // measure the light controller sees via /planning/global_path.
    double route_cumulative_s = 0.0;
    std::vector<std::array<float, 3>> turn_segments;
    for (std::size_t route_index = 0U; route_index < route_lanelets.size(); ++route_index) {
      const auto & route_lanelet = route_lanelets[route_index];
      const bool is_first = route_index == 0U;
      const bool is_last = route_index + 1U == route_lanelets.size();

      double segment_start_s = is_first ? start_match.projection.arc_length : 0.0;
      double segment_end_s = centerlineLength(route_lanelet);
      if (is_last) {
        segment_end_s = goal_match.projection.arc_length;
      }

      if (segment_end_s + 1.0e-6 < segment_start_s) {
        continue;
      }

      const double contributed_length_m = segment_end_s - segment_start_s;
      std::string turn_direction;
      const auto & lanelet_attributes = route_lanelet.attributes();
      const auto turn_attribute = lanelet_attributes.find("turn_direction");
      if (turn_attribute != lanelet_attributes.end()) {
        turn_direction = turn_attribute->second.value();
      }
      const float direction_value =
        turn_direction == "left" ? 1.0F : (turn_direction == "right" ? -1.0F : 0.0F);
      if (direction_value != 0.0F && contributed_length_m > 1.0e-3) {
        const auto window_start = static_cast<float>(route_cumulative_s);
        const auto window_end = static_cast<float>(route_cumulative_s + contributed_length_m);
        if (!turn_segments.empty() &&
          turn_segments.back()[0] == direction_value &&
          window_start - turn_segments.back()[2] < 0.5F)
        {
          // Merge consecutive same-direction lanelets into one signal window.
          turn_segments.back()[2] = window_end;
        } else {
          turn_segments.push_back({direction_value, window_start, window_end});
        }
      }
      route_cumulative_s += contributed_length_m;

      appendCenterlineSegment(route_lanelet, segment_start_s, segment_end_s, path);
    }
    publishRouteTurnSegments(route_cumulative_s, turn_segments);

    if (path.poses.size() < 2U) {
      throw nav2_core::PlannerException("LaneletRoutePlanner generated an empty route");
    }

    const auto geometry_t1 = std::chrono::steady_clock::now();
    updatePathOrientations(path);
    const auto plan_t1 = std::chrono::steady_clock::now();
    const double snap_ms =
      std::chrono::duration<double, std::milli>(snap_t1 - plan_t0).count();
    const double route_ms =
      std::chrono::duration<double, std::milli>(route_t1 - snap_t1).count();
    const double geometry_ms =
      std::chrono::duration<double, std::milli>(geometry_t1 - route_t1).count();
    const double total_ms =
      std::chrono::duration<double, std::milli>(plan_t1 - plan_t0).count();
    RCLCPP_INFO(
      node_->get_logger(),
      "LaneletRoutePlanner plan: start_ll=%ld goal_ll=%ld lanelets=%zu points=%zu "
      "timing snap=%.2fms route=%.2fms geometry=%.2fms total=%.2fms",
      static_cast<long>(start_match.lanelet.id()),
      static_cast<long>(goal_match.lanelet.id()),
      route_lanelets.size(), path.poses.size(),
      snap_ms, route_ms, geometry_ms, total_ms);
    return path;
  }

private:
  template<typename ParameterT>
  void declareIfMissing(const std::string & parameter_name, const ParameterT & value)
  {
    if (!node_->has_parameter(parameter_name)) {
      node_->declare_parameter(parameter_name, value);
    }
  }

  bool loadMap()
  {
    if (map_path_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "LaneletRoutePlanner map_path is empty");
      return false;
    }

    lanelet::GPSPoint origin_gps;
    origin_gps.lat = offset_lat_;
    origin_gps.lon = offset_lon_;
    origin_gps.ele = offset_alt_;
    lanelet::Origin origin(origin_gps);
    lanelet::projection::LocalCartesianProjector projector(origin);

    const auto load_t0 = std::chrono::steady_clock::now();
    try {
      map_ = lanelet::load(map_path_, projector);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_->get_logger(), "LaneletRoutePlanner map load exception: %s",
        exception.what());
      return false;
    }
    const auto load_t1 = std::chrono::steady_clock::now();
    const double load_ms =
      std::chrono::duration<double, std::milli>(load_t1 - load_t0).count();
    RCLCPP_INFO(
      node_->get_logger(),
      "LaneletRoutePlanner map load timing: %.2fms path=%s",
      load_ms, map_path_.c_str());
    return static_cast<bool>(map_);
  }

  bool buildRoutingGraph()
  {
    if (!map_) {
      return false;
    }

    const auto try_build = [this](const std::string & participant) {
        try {
          traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
            routing_location_, participant);
          if (!traffic_rules_) {
            return false;
          }
          routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);
          return static_cast<bool>(routing_graph_);
        } catch (const std::exception & exception) {
          RCLCPP_WARN(
            node_->get_logger(),
            "LaneletRoutePlanner routing graph build failed participant=%s: %s",
            participant.c_str(), exception.what());
          routing_graph_.reset();
          return false;
        }
      };

    const auto graph_t0 = std::chrono::steady_clock::now();
    if (try_build(routing_participant_)) {
      const auto graph_t1 = std::chrono::steady_clock::now();
      const double graph_ms =
        std::chrono::duration<double, std::milli>(graph_t1 - graph_t0).count();
      RCLCPP_INFO(
        node_->get_logger(),
        "LaneletRoutePlanner routing graph timing: %.2fms participant=%s",
        graph_ms, routing_participant_.c_str());
      return true;
    }
    if (routing_participant_ != "vehicle" && try_build("vehicle")) {
      const auto graph_t1 = std::chrono::steady_clock::now();
      const double graph_ms =
        std::chrono::duration<double, std::milli>(graph_t1 - graph_t0).count();
      RCLCPP_WARN(
        node_->get_logger(),
        "LaneletRoutePlanner fallback routing participant applied: %s -> vehicle (%.2fms)",
        routing_participant_.c_str(), graph_ms);
      routing_participant_ = "vehicle";
      return true;
    }
    return false;
  }

  bool initializeMapAndRoutingGraph(std::string & error_message)
  {
    if (!loadMap()) {
      error_message = "map load failed";
      return false;
    }
    if (!buildRoutingGraph()) {
      error_message = "routing graph build failed";
      return false;
    }
    return true;
  }

  void logReady() const
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "LaneletRoutePlanner ready: map=%s frame=%s lane_changes=%s resolution=%.2fm",
      map_path_.c_str(), frame_id_.c_str(),
      allow_lane_changes_ ? "true" : "false", interpolation_resolution_m_);
  }

  void completeInitialization(const bool success, const std::string & error_message)
  {
    {
      std::lock_guard<std::mutex> lock(initialization_mutex_);
      initialization_success_ = success;
      initialization_error_ = error_message;
      initialization_complete_ = true;
    }
    initialization_cv_.notify_all();
  }

  void runInitialization()
  {
    std::string error_message;
    bool success = false;
    try {
      success = initializeMapAndRoutingGraph(error_message);
    } catch (const std::exception & exception) {
      error_message = exception.what();
      success = false;
    }

    completeInitialization(success, error_message);
    if (success) {
      logReady();
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "LaneletRoutePlanner initialization failed: %s",
        error_message.c_str());
    }
  }

  void startInitialization()
  {
    {
      std::lock_guard<std::mutex> lock(initialization_mutex_);
      initialization_started_ = true;
      initialization_complete_ = false;
      initialization_success_ = false;
      initialization_error_.clear();
    }

    if (!async_initialization_) {
      runInitialization();
      if (!waitForInitialization()) {
        throw nav2_core::PlannerException(
                "LaneletRoutePlanner failed to initialize map/routing graph");
      }
      return;
    }

    // HH_260708: Build the heavy Lanelet2 map/routing graph in the background
    // so Nav2 lifecycle activation is not blocked by the full C-track graph.
    initialization_thread_ = std::thread([this]() {runInitialization();});
    RCLCPP_INFO(
      node_->get_logger(),
      "LaneletRoutePlanner async initialization started: map=%s participant=%s wait_timeout=%.1fs",
      map_path_.c_str(), routing_participant_.c_str(),
      async_initialization_plan_wait_timeout_s_);
  }

  bool waitForInitialization()
  {
    std::unique_lock<std::mutex> lock(initialization_mutex_);
    if (!initialization_started_) {
      RCLCPP_ERROR(node_->get_logger(), "LaneletRoutePlanner initialization was not started");
      return false;
    }

    if (!initialization_complete_) {
      if (async_initialization_plan_wait_timeout_s_ <= 0.0) {
        initialization_cv_.wait(lock, [this]() {return initialization_complete_;});
      } else {
        const auto timeout = std::chrono::duration<double>(
          async_initialization_plan_wait_timeout_s_);
        const bool ready = initialization_cv_.wait_for(
          lock, timeout, [this]() {return initialization_complete_;});
        if (!ready) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "LaneletRoutePlanner initialization timed out after %.1fs",
            async_initialization_plan_wait_timeout_s_);
          return false;
        }
      }
    }

    if (!initialization_success_) {
      RCLCPP_ERROR(
        node_->get_logger(), "LaneletRoutePlanner initialization failed: %s",
        initialization_error_.c_str());
      return false;
    }
    return static_cast<bool>(map_) && static_cast<bool>(routing_graph_);
  }

  void joinInitializationThread()
  {
    if (initialization_thread_.joinable()) {
      initialization_thread_.join();
    }
  }

  bool pointInsideLanelet(
    const lanelet::ConstLanelet & lanelet, const double x,
    const double y) const
  {
    const auto left_bound = lanelet.leftBound();
    const auto right_bound = lanelet.rightBound();
    if (left_bound.size() < 2U || right_bound.size() < 2U) {
      return false;
    }

    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(left_bound.size() + right_bound.size());
    for (const auto & point : left_bound) {
      polygon.emplace_back(point.x(), point.y());
    }
    for (std::size_t point_index = right_bound.size(); point_index-- > 0U; ) {
      polygon.emplace_back(right_bound[point_index].x(), right_bound[point_index].y());
    }
    return pointInPolygon2D(polygon, x, y);
  }

  bool laneletBoundsWithinSnapDistance(
    const lanelet::ConstLanelet & lanelet,
    const double x,
    const double y) const
  {
    const auto left_bound = lanelet.leftBound();
    const auto right_bound = lanelet.rightBound();
    if (left_bound.empty() && right_bound.empty()) {
      return false;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    const auto update_bounds = [&](const lanelet::ConstLineString3d & bound) {
        for (const auto & point : bound) {
          min_x = std::min(min_x, point.x());
          min_y = std::min(min_y, point.y());
          max_x = std::max(max_x, point.x());
          max_y = std::max(max_y, point.y());
        }
      };
    update_bounds(left_bound);
    update_bounds(right_bound);

    const double margin = max_snap_distance_m_;
    return x >= min_x - margin && x <= max_x + margin &&
           y >= min_y - margin && y <= max_y + margin;
  }

  ProjectedPoint projectToCenterline(
    const lanelet::ConstLanelet & lanelet, const double x, const double y) const
  {
    ProjectedPoint best_projection;
    const auto & centerline = lanelet.centerline();
    if (centerline.size() < 2U) {
      return best_projection;
    }

    double accumulated_s = 0.0;
    for (std::size_t point_index = 0U; point_index + 1U < centerline.size(); ++point_index) {
      const auto point0 = centerline[point_index];
      const auto point1 = centerline[point_index + 1U];
      const double segment_dx = point1.x() - point0.x();
      const double segment_dy = point1.y() - point0.y();
      const double segment_dz = point1.z() - point0.z();
      const double segment_length = std::hypot(segment_dx, segment_dy);
      if (segment_length < 1.0e-6) {
        continue;
      }

      const double query_dx = x - point0.x();
      const double query_dy = y - point0.y();
      const double segment_t = std::clamp(
        (query_dx * segment_dx + query_dy * segment_dy) / (segment_length * segment_length),
        0.0, 1.0);
      const double projected_x = point0.x() + segment_t * segment_dx;
      const double projected_y = point0.y() + segment_t * segment_dy;
      const double distance_x = x - projected_x;
      const double distance_y = y - projected_y;
      const double distance_sq = distance_x * distance_x + distance_y * distance_y;
      if (distance_sq < best_projection.distance_sq) {
        best_projection.x = projected_x;
        best_projection.y = projected_y;
        best_projection.z = point0.z() + segment_t * segment_dz;
        best_projection.heading = std::atan2(segment_dy, segment_dx);
        best_projection.arc_length = accumulated_s + segment_t * segment_length;
        best_projection.distance_sq = distance_sq;
        best_projection.valid = true;
      }
      accumulated_s += segment_length;
    }
    return best_projection;
  }

  LaneletMatch findBestLanelet(const double x, const double y) const
  {
    LaneletMatch best_inside;
    LaneletMatch best_near;
    const double max_snap_distance_sq = max_snap_distance_m_ * max_snap_distance_m_;

    for (const auto & lanelet : map_->laneletLayer) {
      // HH_260629: Avoid forcing lazy centerline generation for every lanelet
      // during start/goal snapping. Bounds are loaded cheaply and reject most
      // unrelated or opposite-side lanelets before projection.
      if (!laneletBoundsWithinSnapDistance(lanelet, x, y)) {
        continue;
      }
      const ProjectedPoint projection = projectToCenterline(lanelet, x, y);
      if (!projection.valid || projection.distance_sq > max_snap_distance_sq) {
        continue;
      }

      LaneletMatch candidate;
      candidate.lanelet = lanelet;
      candidate.projection = projection;
      candidate.inside = pointInsideLanelet(lanelet, x, y);
      candidate.valid = true;

      if (candidate.inside) {
        if (!best_inside.valid ||
          candidate.projection.distance_sq < best_inside.projection.distance_sq)
        {
          best_inside = candidate;
        }
      } else if (!best_near.valid ||
        candidate.projection.distance_sq < best_near.projection.distance_sq)
      {
        best_near = candidate;
      }
    }

    return best_inside.valid ? best_inside : best_near;
  }

  std::vector<lanelet::ConstLanelet> findRoute(
    const LaneletMatch & start_match,
    const LaneletMatch & goal_match) const
  {
    const bool same_lanelet = start_match.lanelet.id() == goal_match.lanelet.id();
    const bool goal_is_behind_on_same_lane =
      same_lanelet &&
      goal_match.projection.arc_length + same_lane_forward_epsilon_m_ <
      start_match.projection.arc_length;

    if (!goal_is_behind_on_same_lane) {
      const auto route = routing_graph_->shortestPath(
        start_match.lanelet, goal_match.lanelet, 0, allow_lane_changes_);
      if (route) {
        return toVector(*route);
      }
    }

    // HH_260619 - If a goal lies behind the robot on the same one-way lane,
    // do not plan a backward centerline segment. Route through the next
    // connected lanelet so the vehicle keeps legal lane direction.
    std::vector<lanelet::ConstLanelet> best_route;
    double best_route_length = std::numeric_limits<double>::max();
    for (const auto & following_lanelet : routing_graph_->following(
        start_match.lanelet, allow_lane_changes_))
    {
      const auto route = routing_graph_->shortestPath(
        following_lanelet, goal_match.lanelet, 0, allow_lane_changes_);
      if (!route) {
        continue;
      }

      std::vector<lanelet::ConstLanelet> candidate_route;
      candidate_route.push_back(start_match.lanelet);
      const auto tail_route = toVector(*route);
      candidate_route.insert(candidate_route.end(), tail_route.begin(), tail_route.end());
      const double route_length = estimateRouteLength(candidate_route);
      if (route_length < best_route_length) {
        best_route = std::move(candidate_route);
        best_route_length = route_length;
      }
    }
    return best_route;
  }

  static std::vector<lanelet::ConstLanelet> toVector(
    const lanelet::routing::LaneletPath & lanelet_path)
  {
    std::vector<lanelet::ConstLanelet> output;
    output.reserve(lanelet_path.size());
    for (const auto & lanelet : lanelet_path) {
      output.push_back(lanelet);
    }
    return output;
  }

  double estimateRouteLength(const std::vector<lanelet::ConstLanelet> & route_lanelets) const
  {
    double route_length = 0.0;
    for (const auto & lanelet : route_lanelets) {
      route_length += centerlineLength(lanelet);
    }
    return route_length;
  }

  double centerlineLength(const lanelet::ConstLanelet & lanelet) const
  {
    const auto & centerline = lanelet.centerline();
    double length = 0.0;
    for (std::size_t point_index = 0U; point_index + 1U < centerline.size(); ++point_index) {
      length += std::hypot(
        centerline[point_index + 1U].x() - centerline[point_index].x(),
        centerline[point_index + 1U].y() - centerline[point_index].y());
    }
    return length;
  }

  void publishRouteLaneletIds(const std::vector<lanelet::ConstLanelet> & route_lanelets) const
  {
    if (!route_lanelet_ids_pub_) {
      return;
    }
    avg_msgs::msg::RouteLaneletIds msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id_;
    msg.lanelet_ids.reserve(route_lanelets.size());
    for (const auto & route_lanelet : route_lanelets) {
      msg.lanelet_ids.push_back(static_cast<int64_t>(route_lanelet.id()));
    }
    route_lanelet_ids_pub_->publish(msg);
  }

  // HH_260720 - Publish named route-turn fields with +1=left and -1=right.
  void publishRouteTurnSegments(
    double total_route_length_m,
    const std::vector<std::array<float, 3>> & turn_segments) const
  {
    if (!route_turn_segments_pub_) {
      return;
    }
    avg_msgs::msg::RouteTurnSegmentArray msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id_;
    msg.total_route_length_m = static_cast<float>(total_route_length_m);
    msg.segments.reserve(turn_segments.size());
    for (const auto & segment : turn_segments) {
      avg_msgs::msg::RouteTurnSegment typed_segment;
      typed_segment.direction = static_cast<int8_t>(std::lround(segment[0]));
      typed_segment.start_distance_m = segment[1];
      typed_segment.end_distance_m = segment[2];
      msg.segments.push_back(typed_segment);
    }
    route_turn_segments_pub_->publish(msg);
  }

  ProjectedPoint pointAtArcLength(
    const lanelet::ConstLanelet & lanelet,
    const double target_s) const
  {
    ProjectedPoint output;
    const auto & centerline = lanelet.centerline();
    if (centerline.size() < 2U) {
      return output;
    }

    double accumulated_s = 0.0;
    for (std::size_t point_index = 0U; point_index + 1U < centerline.size(); ++point_index) {
      const auto point0 = centerline[point_index];
      const auto point1 = centerline[point_index + 1U];
      const double segment_dx = point1.x() - point0.x();
      const double segment_dy = point1.y() - point0.y();
      const double segment_dz = point1.z() - point0.z();
      const double segment_length = std::hypot(segment_dx, segment_dy);
      if (segment_length < 1.0e-6) {
        continue;
      }
      if (target_s <= accumulated_s + segment_length || point_index + 2U == centerline.size()) {
        const double segment_t = std::clamp(
          (target_s - accumulated_s) / segment_length, 0.0, 1.0);
        output.x = point0.x() + segment_t * segment_dx;
        output.y = point0.y() + segment_t * segment_dy;
        output.z = point0.z() + segment_t * segment_dz;
        output.heading = std::atan2(segment_dy, segment_dx);
        output.arc_length = accumulated_s + segment_t * segment_length;
        output.valid = true;
        return output;
      }
      accumulated_s += segment_length;
    }
    return output;
  }

  void appendCenterlineSegment(
    const lanelet::ConstLanelet & lanelet,
    const double start_s,
    const double end_s,
    nav_msgs::msg::Path & path) const
  {
    const double clamped_start_s = std::max(0.0, start_s);
    const double clamped_end_s = std::max(clamped_start_s, end_s);

    appendPoint(pointAtArcLength(lanelet, clamped_start_s), path);
    double sample_s = clamped_start_s + interpolation_resolution_m_;
    while (sample_s < clamped_end_s - 1.0e-6) {
      appendPoint(pointAtArcLength(lanelet, sample_s), path);
      sample_s += interpolation_resolution_m_;
    }
    appendPoint(pointAtArcLength(lanelet, clamped_end_s), path);
  }

  void appendPoint(const ProjectedPoint & point, nav_msgs::msg::Path & path) const
  {
    if (!point.valid) {
      return;
    }
    if (!path.poses.empty()) {
      const auto & previous_position = path.poses.back().pose.position;
      if (std::hypot(previous_position.x - point.x, previous_position.y - point.y) < 0.03) {
        return;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    // HH_260707 - Nav2 and RViz consume this as a 2D route. Flatten OSM
    // altitude so plans do not inherit source lanelet negative Z values.
    pose.pose.position.z = flatten_path_z_ ? 0.0 : point.z;
    pose.pose.orientation = yawToQuaternion(point.heading);
    path.poses.push_back(pose);
  }

  void updatePathOrientations(nav_msgs::msg::Path & path) const
  {
    for (std::size_t pose_index = 0U; pose_index + 1U < path.poses.size(); ++pose_index) {
      const auto & current_position = path.poses[pose_index].pose.position;
      const auto & next_position = path.poses[pose_index + 1U].pose.position;
      const double heading = std::atan2(
        next_position.y - current_position.y,
        next_position.x - current_position.x);
      path.poses[pose_index].pose.orientation = yawToQuaternion(heading);
    }
    if (path.poses.size() >= 2U) {
      path.poses.back().pose.orientation = path.poses[path.poses.size() - 2U].pose.orientation;
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::string map_path_;
  std::string frame_id_{"map"};
  std::string routing_location_{"de"};
  std::string routing_participant_{"vehicle:car"};
  double offset_lat_{0.0};
  double offset_lon_{0.0};
  double offset_alt_{0.0};
  double max_snap_distance_m_{8.0};
  double interpolation_resolution_m_{0.20};
  double same_lane_forward_epsilon_m_{0.30};
  bool allow_lane_changes_{true};
  bool flatten_path_z_{true};
  bool async_initialization_{true};
  double async_initialization_plan_wait_timeout_s_{60.0};
  std::string route_lanelet_ids_topic_{"/planning/route_lanelet_ids"};
  std::string route_turn_segments_topic_{"/planning/route_turn_segments"};
  rclcpp_lifecycle::LifecyclePublisher<avg_msgs::msg::RouteLaneletIds>::SharedPtr
    route_lanelet_ids_pub_;
  rclcpp_lifecycle::LifecyclePublisher<avg_msgs::msg::RouteTurnSegmentArray>::SharedPtr
    route_turn_segments_pub_;
  lanelet::LaneletMapPtr map_;
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_;
  lanelet::routing::RoutingGraphUPtr routing_graph_;
  std::thread initialization_thread_;
  std::mutex initialization_mutex_;
  std::condition_variable initialization_cv_;
  bool initialization_started_{false};
  bool initialization_complete_{false};
  bool initialization_success_{false};
  std::string initialization_error_;
};

}  // namespace camrod_planning

PLUGINLIB_EXPORT_CLASS(camrod_planning::LaneletRoutePlanner, nav2_core::GlobalPlanner)
