#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <avg_msgs/msg/avg_planning_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/quaternion.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <rclcpp/rclcpp.hpp>

#include "camrod_map/custom_regulatory_elements.hpp"

namespace
{
struct LoaderConfig
{
  std::string map_path;
  double offset_lat{0.0};
  double offset_lon{0.0};
  double offset_alt{0.0};
};

struct NearestResult
{
  double sq_dist{std::numeric_limits<double>::max()};
  bool valid{false};
  lanelet::ConstPoint3d nearest_point;
  double heading{0.0};
};

// Implements `yawToQuat` behavior.
avg_msgs::msg::Quaternion yawToQuat(double yaw)
{
  avg_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

// Implements `yawToQuatRos` behavior.
geometry_msgs::msg::Quaternion yawToQuatRos(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

// Implements `pointInPolygon2D` behavior.
bool pointInPolygon2D(const std::vector<std::pair<double, double>> & poly, double x, double y)
{
  if (poly.size() < 3) {
    return false;
  }
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double xi = poly[i].first;
    const double yi = poly[i].second;
    const double xj = poly[j].first;
    const double yj = poly[j].second;
    const bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

class GoalSnapperNode : public rclcpp::Node
{
public:
  using AvgPlanningMsgs = avg_msgs::msg::AvgPlanningMsgs;
  using ModuleState = avg_msgs::msg::ModuleState;

  GoalSnapperNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("goal_snapper")
  {
    // HH_260112 Snap RViz goal to nearest lanelet centerline.
    cfg_.map_path = declare_parameter<std::string>("map_path", "");
    cfg_.offset_lat = declare_parameter<double>("offset_lat", 0.0);
    cfg_.offset_lon = declare_parameter<double>("offset_lon", 0.0);
    cfg_.offset_alt = declare_parameter<double>("offset_alt", 0.0);
    input_goal_topic_ = declare_parameter<std::string>("input_goal_topic", "/goal_pose");
    output_goal_topic_ = declare_parameter<std::string>("output_goal_topic", "/planning/goal_pose");
    // HH_260317 Publish ROS-native snapped goal for Nav2 topic compatibility.
    output_goal_topic_ros_ = declare_parameter<std::string>(
      "output_goal_topic_ros", "/planning/goal_pose_snapped_ros");
    publish_planning_status_ = declare_parameter<bool>("publish_planning_status", false);
    planning_status_topic_ =
      declare_parameter<std::string>("planning_status_topic", "/planning/status");
    max_search_radius_ = declare_parameter<double>("max_search_radius", 30.0);
    require_lanelet_containment_ = declare_parameter<bool>("require_lanelet_containment", true);
    fallback_uncontained_ = declare_parameter<bool>("fallback_uncontained", true);
    use_map_z_ = declare_parameter<bool>("use_map_z", true);
    flatten_to_ground_ = declare_parameter<bool>("flatten_to_ground", true);
    map_z_offset_ = declare_parameter<double>("map_z_offset", 0.0);

    // HH_260528 Restrict goal snapping to the lanelet component connected to
    // the robot's current lane pose to prevent cross-network jumps.
    restrict_to_connected_lanelet_component_ = declare_parameter<bool>(
      "restrict_to_connected_lanelet_component", true);
    allow_component_fallback_to_global_ = declare_parameter<bool>(
      "allow_component_fallback_to_global", false);
    component_include_lane_changes_ = declare_parameter<bool>(
      "component_include_lane_changes", true);
    component_max_expansion_nodes_ = declare_parameter<int>(
      "component_max_expansion_nodes", 5000);
    current_pose_topic_ = declare_parameter<std::string>(
      "current_pose_topic", "/planning/lanelet_pose");
    current_lanelet_search_radius_ = declare_parameter<double>(
      "current_lanelet_search_radius", 12.0);
    component_update_min_period_s_ = declare_parameter<double>(
      "component_update_min_period_s", 0.2);
    component_update_min_displacement_m_ = declare_parameter<double>(
      "component_update_min_displacement_m", 0.5);
    routing_location_ = declare_parameter<std::string>("routing_location", "de");
    routing_participant_ = declare_parameter<std::string>(
      "routing_participant", "vehicle:car");

    // HH_260329 Debounce duplicate goal callbacks when the same topic is
    // subscribed by both avg_msgs and geometry_msgs interfaces.
    duplicate_goal_xy_eps_m_ = declare_parameter<double>("duplicate_goal_xy_eps_m", 0.05);
    duplicate_goal_z_eps_m_ = declare_parameter<double>("duplicate_goal_z_eps_m", 0.10);
    duplicate_goal_time_window_s_ = declare_parameter<double>("duplicate_goal_time_window_s", 0.25);

    if (!loadMap()) {
      RCLCPP_FATAL(get_logger(), "goal snapper: failed to load map. exiting.");
      rclcpp::shutdown();
      return;
    }

    if (restrict_to_connected_lanelet_component_) {
      if (!buildRoutingGraph()) {
        if (allow_component_fallback_to_global_) {
          RCLCPP_WARN(
            get_logger(),
            "goal snapper: routing graph unavailable; using global nearest-lanelet fallback");
          restrict_to_connected_lanelet_component_ = false;
        } else {
          RCLCPP_FATAL(
            get_logger(),
            "goal snapper: routing graph unavailable while strict connected-lane mode is enabled");
          rclcpp::shutdown();
          return;
        }
      }
    }

    pub_goal_ = create_publisher<avg_msgs::msg::PoseStamped>(
      output_goal_topic_, rclcpp::QoS(10));
    pub_goal_ros_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_goal_topic_ros_, rclcpp::QoS(10));
    if (publish_planning_status_) {
      pub_avg_planning_ = create_publisher<AvgPlanningMsgs>(
        planning_status_topic_, rclcpp::QoS(10));
    }
    sub_goal_ = create_subscription<avg_msgs::msg::PoseStamped>(
      input_goal_topic_, rclcpp::QoS(10),
      std::bind(&GoalSnapperNode::onGoal, this, std::placeholders::_1));
    sub_goal_ros_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_goal_topic_, rclcpp::QoS(10),
      std::bind(&GoalSnapperNode::onGoalRos, this, std::placeholders::_1));

    if (restrict_to_connected_lanelet_component_) {
      sub_current_pose_ = create_subscription<avg_msgs::msg::PoseStamped>(
        current_pose_topic_, rclcpp::QoS(10),
        std::bind(&GoalSnapperNode::onCurrentPose, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "goal_snapper ready: map=%s input=%s output(avg)=%s output(ros)=%s lane_component=%s",
      cfg_.map_path.c_str(), input_goal_topic_.c_str(),
      output_goal_topic_.c_str(), output_goal_topic_ros_.c_str(),
      restrict_to_connected_lanelet_component_ ? "enabled" : "disabled");
  }

private:
  // Loads `Map` data or configuration.
  bool loadMap()
  {
    lanelet::GPSPoint gps;
    gps.lat = cfg_.offset_lat;
    gps.lon = cfg_.offset_lon;
    gps.ele = cfg_.offset_alt;
    lanelet::Origin origin(gps);
    lanelet::projection::LocalCartesianProjector projector(origin);

    try {
      map_ = lanelet::load(cfg_.map_path, projector);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception loading map %s: %s", cfg_.map_path.c_str(), e.what());
      return false;
    }
    if (!map_) {
      RCLCPP_ERROR(get_logger(), "lanelet::load returned nullptr for %s", cfg_.map_path.c_str());
      return false;
    }
    map_ground_z_ = computeGroundZ(*map_);
    RCLCPP_DEBUG(get_logger(), "map ground z (median)=%.3f", map_ground_z_);
    return true;
  }

  // Builds routing graph used for connected-lanelet component extraction.
  bool buildRoutingGraph()
  {
    if (!map_) {
      return false;
    }

    auto try_build = [this](const std::string & participant) -> bool {
        try {
          traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
            routing_location_, participant);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            get_logger(),
            "goal_snapper: traffic rule create failed (location=%s participant=%s): %s",
            routing_location_.c_str(), participant.c_str(), e.what());
          return false;
        }

        if (!traffic_rules_) {
          return false;
        }

        try {
          routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            get_logger(),
            "goal_snapper: routing graph build failed (location=%s participant=%s): %s",
            routing_location_.c_str(), participant.c_str(), e.what());
          routing_graph_.reset();
          return false;
        }

        return static_cast<bool>(routing_graph_);
      };

    // HH_260528 Try configured participant first, then generic vehicle fallback.
    if (try_build(routing_participant_)) {
      return true;
    }

    const std::string generic_vehicle = "vehicle";
    if (routing_participant_ != generic_vehicle && try_build(generic_vehicle)) {
      RCLCPP_WARN(
        get_logger(),
        "goal_snapper: fallback routing participant applied: %s -> %s",
        routing_participant_.c_str(), generic_vehicle.c_str());
      routing_participant_ = generic_vehicle;
      return true;
    }

    return false;
  }

  // Handles current lanelet pose updates for connected-component filtering.
  void onCurrentPose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!restrict_to_connected_lanelet_component_ || !routing_graph_) {
      return;
    }

    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double now_sec = this->get_clock()->now().seconds();

    if (has_last_component_pose_) {
      const double dt = now_sec - last_component_update_sec_;
      const double moved = std::hypot(px - last_component_pose_x_, py - last_component_pose_y_);
      const bool skip_by_time = component_update_min_period_s_ > 0.0 && dt < component_update_min_period_s_;
      const bool skip_by_movement =
        component_update_min_displacement_m_ > 0.0 && moved < component_update_min_displacement_m_;
      if (skip_by_time && skip_by_movement) {
        return;
      }
    }

    lanelet::ConstLanelet seed;
    if (!findBestLaneletForPoint(px, py, seed)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: current pose is not near any lanelet (r=%.1fm), keep previous connected component",
        current_lanelet_search_radius_);
      return;
    }

    rebuildConnectedComponent(seed);

    has_last_component_pose_ = true;
    last_component_pose_x_ = px;
    last_component_pose_y_ = py;
    last_component_update_sec_ = now_sec;
  }

  // Recomputes connected lanelet id set from the given seed lanelet.
  void rebuildConnectedComponent(const lanelet::ConstLanelet & seed_lanelet)
  {
    if (!routing_graph_) {
      return;
    }

    std::unordered_set<lanelet::Id> new_ids;
    std::deque<lanelet::ConstLanelet> queue;

    new_ids.insert(seed_lanelet.id());
    queue.push_back(seed_lanelet);

    int expanded_nodes = 0;
    while (!queue.empty()) {
      if (component_max_expansion_nodes_ > 0 && expanded_nodes >= component_max_expansion_nodes_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "goal_snapper: connected component expansion capped at %d nodes",
          component_max_expansion_nodes_);
        break;
      }

      const lanelet::ConstLanelet current = queue.front();
      queue.pop_front();
      ++expanded_nodes;

      auto enqueue_neighbors = [&new_ids, &queue](const lanelet::ConstLanelets & neighbors) {
          for (const auto & nb : neighbors) {
            const lanelet::Id id = nb.id();
            if (new_ids.insert(id).second) {
              queue.push_back(nb);
            }
          }
        };

      enqueue_neighbors(routing_graph_->following(current, component_include_lane_changes_));
      enqueue_neighbors(routing_graph_->previous(current, component_include_lane_changes_));
    }

    const bool seed_changed = (!has_component_seed_) || (seed_lanelet.id() != component_seed_lanelet_id_);
    component_seed_lanelet_id_ = seed_lanelet.id();
    has_component_seed_ = true;
    connected_lanelet_ids_ = std::move(new_ids);

    if (seed_changed) {
      RCLCPP_INFO(
        get_logger(),
        "goal_snapper: connected lanelet component updated seed=%ld size=%zu",
        static_cast<long>(component_seed_lanelet_id_), connected_lanelet_ids_.size());
    }
  }

  // Finds lanelet for current pose prioritizing containment, then nearest centerline.
  bool findBestLaneletForPoint(double x, double y, lanelet::ConstLanelet & out_lanelet) const
  {
    if (!map_) {
      return false;
    }

    const double max_sq = current_lanelet_search_radius_ * current_lanelet_search_radius_;

    bool found_inside = false;
    lanelet::ConstLanelet best_inside;
    double best_inside_sq = std::numeric_limits<double>::max();

    lanelet::ConstLanelet best_near;
    double best_near_sq = std::numeric_limits<double>::max();

    for (const auto & ll : map_->laneletLayer) {
      const double d2 = distanceSqToCenterline(ll, x, y);
      if (!(d2 < max_sq)) {
        continue;
      }

      if (pointInsideLanelet(ll, x, y)) {
        if (d2 < best_inside_sq) {
          best_inside_sq = d2;
          best_inside = ll;
          found_inside = true;
        }
      } else if (d2 < best_near_sq) {
        best_near_sq = d2;
        best_near = ll;
      }
    }

    if (found_inside) {
      out_lanelet = best_inside;
      return true;
    }
    if (best_near_sq < std::numeric_limits<double>::max()) {
      out_lanelet = best_near;
      return true;
    }
    return false;
  }

  // Returns squared distance from a 2D point to a lanelet centerline.
  double distanceSqToCenterline(const lanelet::ConstLanelet & ll, double x, double y) const
  {
    const auto & cl = ll.centerline();
    if (cl.size() < 2) {
      return std::numeric_limits<double>::max();
    }

    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < cl.size(); ++i) {
      const auto p0 = cl[i];
      const auto p1 = cl[i + 1];
      const double vx = p1.x() - p0.x();
      const double vy = p1.y() - p0.y();
      const double wx = x - p0.x();
      const double wy = y - p0.y();
      const double seg_len2 = vx * vx + vy * vy + 1e-6;
      double t = (vx * wx + vy * wy) / seg_len2;
      t = std::max(0.0, std::min(1.0, t));
      const double proj_x = p0.x() + t * vx;
      const double proj_y = p0.y() + t * vy;
      const double dx = x - proj_x;
      const double dy = y - proj_y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best) {
        best = d2;
      }
    }
    return best;
  }

  // Handles the `onGoal` callback.
  void onGoal(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;
    if (shouldSkipDuplicateGoal(msg->header.frame_id, msg->header.stamp, px, py, pz)) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 500,
        "goal_snapper: skipped duplicate avg goal in frame=%s",
        msg->header.frame_id.c_str());
      return;
    }

    NearestResult nearest;
    double snapped_z = 0.0;
    if (!snapGoal(px, py, pz, nearest, snapped_z)) {
      return;
    }

    avg_msgs::msg::PoseStamped out;
    out.header = msg->header;
    out.pose.position.x = nearest.nearest_point.x();
    out.pose.position.y = nearest.nearest_point.y();
    out.pose.position.z = snapped_z;
    out.pose.orientation = yawToQuat(nearest.heading);
    pub_goal_->publish(out);
    publishRosGoal(out);
    // HH_260316 Keep explicit runtime trace for path-failure diagnosis.
    // This confirms whether BT/NavigateToPose is using snapped goal coordinates.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "goal_snapper: in=(%.2f, %.2f) out=(%.2f, %.2f) dist=%.2f",
      px, py, out.pose.position.x, out.pose.position.y, std::sqrt(nearest.sq_dist));
    publishAvgPlanning(out);
  }

  // Handles the `onGoalRos` callback.
  void onGoalRos(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    // HH_260317 Accept RViz 2D Goal Pose directly (geometry_msgs).
    // Internal planning helpers still consume avg_msgs output_goal_topic_.
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;
    if (shouldSkipDuplicateGoal(msg->header.frame_id, msg->header.stamp, px, py, pz)) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 500,
        "goal_snapper: skipped duplicate ros goal in frame=%s",
        msg->header.frame_id.c_str());
      return;
    }

    NearestResult nearest;
    double snapped_z = 0.0;
    if (!snapGoal(px, py, pz, nearest, snapped_z)) {
      return;
    }

    avg_msgs::msg::PoseStamped out_avg;
    out_avg.header.stamp = msg->header.stamp;
    out_avg.header.frame_id = msg->header.frame_id;
    out_avg.pose.position.x = nearest.nearest_point.x();
    out_avg.pose.position.y = nearest.nearest_point.y();
    out_avg.pose.position.z = snapped_z;
    out_avg.pose.orientation = yawToQuat(nearest.heading);
    pub_goal_->publish(out_avg);

    geometry_msgs::msg::PoseStamped out_ros;
    out_ros.header = msg->header;
    out_ros.pose.position.x = nearest.nearest_point.x();
    out_ros.pose.position.y = nearest.nearest_point.y();
    out_ros.pose.position.z = snapped_z;
    out_ros.pose.orientation = yawToQuatRos(nearest.heading);
    pub_goal_ros_->publish(out_ros);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "goal_snapper(ros): in=(%.2f, %.2f) out=(%.2f, %.2f) dist=%.2f",
      px, py, out_ros.pose.position.x, out_ros.pose.position.y, std::sqrt(nearest.sq_dist));
    publishAvgPlanning(out_avg);
  }

  // Implements `snapGoal` behavior.
  bool snapGoal(
    const double px, const double py, const double pz,
    NearestResult & nearest, double & snapped_z)
  {
    nearest = findNearestCenterline(px, py, require_lanelet_containment_);
    if (!nearest.valid && require_lanelet_containment_ && fallback_uncontained_) {
      nearest = findNearestCenterline(px, py, false);
      if (nearest.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Goal is outside lanelet polygon; snapped to nearest centerline");
      }
    }
    if (!nearest.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper reject: in=(%.2f, %.2f) reason=outside_lanelet_or_far(>%.1fm)",
        px, py, max_search_radius_);
      return false;
    }
    snapped_z = use_map_z_ ? (nearest.nearest_point.z() + map_z_offset_) : pz;
    if (flatten_to_ground_) {
      snapped_z = map_ground_z_ + map_z_offset_;
    }
    return true;
  }

  // Publishes `RosGoal` output.
  void publishRosGoal(const avg_msgs::msg::PoseStamped & goal_pose)
  {
    if (!pub_goal_ros_) {
      return;
    }
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = goal_pose.header.stamp;
    out.header.frame_id = goal_pose.header.frame_id;
    out.pose.position.x = goal_pose.pose.position.x;
    out.pose.position.y = goal_pose.pose.position.y;
    out.pose.position.z = goal_pose.pose.position.z;
    out.pose.orientation.x = goal_pose.pose.orientation.x;
    out.pose.orientation.y = goal_pose.pose.orientation.y;
    out.pose.orientation.z = goal_pose.pose.orientation.z;
    out.pose.orientation.w = goal_pose.pose.orientation.w;
    pub_goal_ros_->publish(out);
  }

  // Publishes `AvgPlanning` output.
  void publishAvgPlanning(const avg_msgs::msg::PoseStamped & goal_pose)
  {
    if (!publish_planning_status_ || !pub_avg_planning_) {
      return;
    }
    AvgPlanningMsgs msg;
    msg.stamp = now();
    msg.state.stamp = msg.stamp;
    msg.state.module_name = "planning";
    msg.state.level = ModuleState::OK;
    msg.state.message = "goal_snapper";
    msg.goal_pose = goal_pose;
    pub_avg_planning_->publish(msg);
  }

  // Skips duplicate goals that arrive from dual subscriptions on the same topic.
  bool shouldSkipDuplicateGoal(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    double x, double y, double z)
  {
    const double now_sec = this->get_clock()->now().seconds();
    if (has_last_goal_) {
      const bool same_frame = (frame_id == last_goal_frame_id_);
      const double dxy = std::hypot(x - last_goal_x_, y - last_goal_y_);
      const double dz = std::fabs(z - last_goal_z_);
      const bool close_pose =
        (dxy <= duplicate_goal_xy_eps_m_) && (dz <= duplicate_goal_z_eps_m_);
      const bool same_stamp =
        (stamp.sec == last_goal_stamp_sec_) && (stamp.nanosec == last_goal_stamp_nanosec_);
      const bool near_time = (now_sec - last_goal_received_sec_) <= duplicate_goal_time_window_s_;
      if (same_frame && close_pose && (same_stamp || near_time)) {
        return true;
      }
    }

    has_last_goal_ = true;
    last_goal_frame_id_ = frame_id;
    last_goal_stamp_sec_ = stamp.sec;
    last_goal_stamp_nanosec_ = stamp.nanosec;
    last_goal_x_ = x;
    last_goal_y_ = y;
    last_goal_z_ = z;
    last_goal_received_sec_ = now_sec;
    return false;
  }

  // Implements `findNearestCenterline` behavior.
  NearestResult findNearestCenterline(
    double x, double y, bool require_lanelet_containment)
  {
    NearestResult best;
    const double max_sq = max_search_radius_ * max_search_radius_;

    const bool strict_component =
      restrict_to_connected_lanelet_component_ && !allow_component_fallback_to_global_;
    const bool component_ready = !connected_lanelet_ids_.empty();
    if (strict_component && !component_ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: connected lanelet component is not ready yet; reject goal");
      return best;
    }

    for (const auto & ll : map_->laneletLayer) {
      if (restrict_to_connected_lanelet_component_ && component_ready) {
        if (connected_lanelet_ids_.count(ll.id()) == 0U) {
          continue;
        }
      }

      if (require_lanelet_containment && !pointInsideLanelet(ll, x, y)) {
        continue;
      }
      const auto & cl = ll.centerline();
      if (cl.size() < 2) {
        continue;
      }
      for (size_t i = 0; i + 1 < cl.size(); ++i) {
        const auto p0 = cl[i];
        const auto p1 = cl[i + 1];
        const double vx = p1.x() - p0.x();
        const double vy = p1.y() - p0.y();
        const double wx = x - p0.x();
        const double wy = y - p0.y();
        const double seg_len2 = vx * vx + vy * vy + 1e-6;
        double t = (vx * wx + vy * wy) / seg_len2;
        t = std::max(0.0, std::min(1.0, t));
        const double proj_x = p0.x() + t * vx;
        const double proj_y = p0.y() + t * vy;
        const double dx = x - proj_x;
        const double dy = y - proj_y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < best.sq_dist && dist2 < max_sq) {
          best.sq_dist = dist2;
          best.valid = true;
          const double proj_z = p0.z() + t * (p1.z() - p0.z());
          best.nearest_point = lanelet::Point3d(lanelet::InvalId, proj_x, proj_y, proj_z);
          best.heading = std::atan2(vy, vx);
        }
      }
    }

    if (!best.valid && restrict_to_connected_lanelet_component_ && !component_ready &&
      allow_component_fallback_to_global_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: connected component unavailable; global fallback active");
    }

    return best;
  }

  // Implements `pointInsideLanelet` behavior.
  bool pointInsideLanelet(const lanelet::ConstLanelet & ll, double x, double y) const
  {
    const auto left = ll.leftBound();
    const auto right = ll.rightBound();
    if (left.size() < 2 || right.size() < 2) {
      return false;
    }
    std::vector<std::pair<double, double>> poly;
    poly.reserve(left.size() + right.size());
    for (const auto & pt : left) {
      poly.emplace_back(pt.x(), pt.y());
    }
    for (size_t i = right.size(); i-- > 0;) {
      poly.emplace_back(right[i].x(), right[i].y());
    }
    return pointInPolygon2D(poly, x, y);
  }

  // Computes `GroundZ` values.
  double computeGroundZ(const lanelet::LaneletMap & map)
  {
    std::vector<double> zs;
    zs.reserve(map.pointLayer.size());
    for (const auto & pt : map.pointLayer) {
      zs.push_back(pt.z());
    }
    if (zs.empty()) {
      return 0.0;
    }
    std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
    return zs[zs.size() / 2];
  }

  LoaderConfig cfg_;
  lanelet::LaneletMapPtr map_;

  std::string input_goal_topic_;
  std::string output_goal_topic_;
  std::string output_goal_topic_ros_;
  std::string planning_status_topic_;
  double max_search_radius_{30.0};
  bool require_lanelet_containment_{true};
  bool fallback_uncontained_{true};
  bool use_map_z_{true};
  bool flatten_to_ground_{true};
  double map_z_offset_{0.0};
  double map_ground_z_{0.0};
  double duplicate_goal_xy_eps_m_{0.05};
  double duplicate_goal_z_eps_m_{0.10};
  double duplicate_goal_time_window_s_{0.25};
  bool has_last_goal_{false};
  std::string last_goal_frame_id_;
  int32_t last_goal_stamp_sec_{0};
  uint32_t last_goal_stamp_nanosec_{0};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  double last_goal_z_{0.0};
  double last_goal_received_sec_{0.0};

  // HH_260528 Connected lanelet-component restriction state.
  bool restrict_to_connected_lanelet_component_{true};
  bool allow_component_fallback_to_global_{false};
  bool component_include_lane_changes_{true};
  int component_max_expansion_nodes_{5000};
  std::string current_pose_topic_;
  double current_lanelet_search_radius_{12.0};
  double component_update_min_period_s_{0.2};
  double component_update_min_displacement_m_{0.5};
  std::string routing_location_{"de"};
  std::string routing_participant_{"vehicle:car"};
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_;
  lanelet::routing::RoutingGraphUPtr routing_graph_;
  std::unordered_set<lanelet::Id> connected_lanelet_ids_;
  lanelet::Id component_seed_lanelet_id_{lanelet::InvalId};
  bool has_component_seed_{false};
  bool has_last_component_pose_{false};
  double last_component_pose_x_{0.0};
  double last_component_pose_y_{0.0};
  double last_component_update_sec_{0.0};

  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_ros_;
  rclcpp::Publisher<AvgPlanningMsgs>::SharedPtr pub_avg_planning_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_ros_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_current_pose_;
  bool publish_planning_status_{false};
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSnapperNode>());
  rclcpp::shutdown();
  return 0;
}
