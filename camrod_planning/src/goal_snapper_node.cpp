#include <algorithm>
#include <chrono>
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
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
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
    // HH_260617: Keep snapped goals inside the same traversable raster component
    // used by Nav2. Lanelet routing may be connected while the OccupancyGrid is
    // still disconnected by map gaps, which causes "no valid path" at runtime.
    restrict_to_cost_grid_component_ = declare_parameter<bool>(
      "restrict_to_cost_grid_component", true);
    allow_cost_grid_component_fallback_ = declare_parameter<bool>(
      "allow_cost_grid_component_fallback", false);
    // HH_260619 - UI campsite centers are intentionally outside lanelets. If a
    // stale connected/cost component would snap that off-lane goal to a much
    // farther lanelet, prefer a bounded global nearest-centerline snap instead.
    uncontained_global_snap_override_enable_ = declare_parameter<bool>(
      "uncontained_global_snap_override_enable", true);
    uncontained_global_snap_max_distance_m_ = declare_parameter<double>(
      "uncontained_global_snap_max_distance_m", 15.0);
    uncontained_global_snap_min_improvement_m_ = declare_parameter<double>(
      "uncontained_global_snap_min_improvement_m", 5.0);
    cost_grid_topic_ = declare_parameter<std::string>(
      "cost_grid_topic", "/map/cost_grid/lanelet");
    cost_grid_block_threshold_ = std::clamp(
      static_cast<int>(declare_parameter<int>("cost_grid_block_threshold", 100)), 1, 101);

    // HH_260329 Debounce duplicate goal callbacks when the same topic is
    // subscribed by both avg_msgs and geometry_msgs interfaces.
    duplicate_goal_xy_eps_m_ = declare_parameter<double>("duplicate_goal_xy_eps_m", 0.05);
    duplicate_goal_z_eps_m_ = declare_parameter<double>("duplicate_goal_z_eps_m", 0.10);
    duplicate_goal_time_window_s_ = declare_parameter<double>("duplicate_goal_time_window_s", 0.25);
    // HH_260618: Default to latest-wins command semantics. RViz/UI goal clicks
    // should preempt stale active goals; enable sequential release only for
    // explicit waypoint smoke tests.
    sequential_goal_release_enable_ =
      declare_parameter<bool>("sequential_goal_release_enable", false);
    sequential_goal_queue_policy_ =
      declare_parameter<std::string>("sequential_goal_queue_policy", "replace_pending");
    sequential_goal_reached_distance_m_ =
      declare_parameter<double>("sequential_goal_reached_distance_m", 0.8);
    sequential_goal_stop_hold_s_ =
      declare_parameter<double>("sequential_goal_stop_hold_s", 0.5);
    sequential_goal_duplicate_xy_eps_m_ =
      declare_parameter<double>("sequential_goal_duplicate_xy_eps_m", 0.20);
    sequential_goal_max_queue_size_ =
      std::max(1, static_cast<int>(declare_parameter<int>("sequential_goal_max_queue_size", 10)));
    sequential_goal_status_topic_ = declare_parameter<std::string>(
      "sequential_goal_status_topic", "/planning/navigate_to_pose/_action/status");
    // HH_260618: Wait for Nav2's terminal SUCCEEDED status before releasing the
    // next queued waypoint. Pose-distance alone can fire while BT is still
    // finishing, which causes topic-goal preemption instead of a clean stop.
    sequential_goal_require_nav2_terminal_ =
      declare_parameter<bool>("sequential_goal_require_nav2_terminal", true);
    // HH_260619 - If the robot pose is manually reset or localization jumps while
    // a NavigateToPose action is active, re-send the current snapped goal once.
    // This forces BT/Navigator to rebuild its blackboard path from the new pose
    // instead of rotating in place against the stale FollowPath context.
    reissue_active_goal_on_pose_jump_ =
      declare_parameter<bool>("reissue_active_goal_on_pose_jump", true);
    pose_jump_reissue_distance_m_ =
      declare_parameter<double>("pose_jump_reissue_distance_m", 1.5);
    pose_jump_reissue_min_interval_s_ =
      declare_parameter<double>("pose_jump_reissue_min_interval_s", 1.0);
    if (
      sequential_goal_queue_policy_ != "append" &&
      sequential_goal_queue_policy_ != "replace_pending" &&
      sequential_goal_queue_policy_ != "drop_if_busy")
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid sequential_goal_queue_policy='%s'; using replace_pending",
        sequential_goal_queue_policy_.c_str());
      sequential_goal_queue_policy_ = "replace_pending";
    }

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

    if (
      restrict_to_connected_lanelet_component_ ||
      restrict_to_cost_grid_component_ ||
      sequential_goal_release_enable_ ||
      reissue_active_goal_on_pose_jump_)
    {
      sub_current_pose_ = create_subscription<avg_msgs::msg::PoseStamped>(
        current_pose_topic_, rclcpp::QoS(10),
        std::bind(&GoalSnapperNode::onCurrentPose, this, std::placeholders::_1));
    }
    if (restrict_to_cost_grid_component_) {
      sub_cost_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        cost_grid_topic_, rclcpp::QoS(1),
        std::bind(&GoalSnapperNode::onCostGrid, this, std::placeholders::_1));
    }
    // HHL_260622 - Subscribe to Nav2 status whenever pose-jump reissue is enabled.
    // Otherwise a succeeded direct goal can stay "active" and be reissued after RViz initialpose resets.
    if (sequential_goal_release_enable_ || reissue_active_goal_on_pose_jump_) {
      sub_nav_status_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        sequential_goal_status_topic_, rclcpp::QoS(10),
        std::bind(&GoalSnapperNode::onNavStatus, this, std::placeholders::_1));
    }
    if (sequential_goal_release_enable_) {
      queue_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GoalSnapperNode::releasePendingGoalIfReady, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "goal_snapper ready: map=%s input=%s output(avg)=%s output(ros)=%s lane_component=%s cost_grid_component=%s sequential_release=%s",
      cfg_.map_path.c_str(), input_goal_topic_.c_str(),
      output_goal_topic_.c_str(), output_goal_topic_ros_.c_str(),
      restrict_to_connected_lanelet_component_ ? "enabled" : "disabled",
      restrict_to_cost_grid_component_ ? "enabled" : "disabled",
      sequential_goal_release_enable_ ? "enabled" : "disabled");
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
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double now_sec = this->get_clock()->now().seconds();
    const bool has_previous_pose = has_pose_jump_check_pose_;
    const double pose_jump_distance = has_previous_pose
      ? std::hypot(px - last_pose_jump_check_x_, py - last_pose_jump_check_y_)
      : 0.0;
    last_pose_jump_check_x_ = px;
    last_pose_jump_check_y_ = py;
    has_pose_jump_check_pose_ = true;

    latest_current_pose_x_ = px;
    latest_current_pose_y_ = py;
    has_latest_current_pose_ = true;
    rebuildCostGridComponent();
    releasePendingGoalIfReady();
    maybeReissueActiveGoalOnPoseJump(pose_jump_distance, now_sec);

    if (!restrict_to_connected_lanelet_component_ || !routing_graph_) {
      return;
    }

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

  // Handles Nav2 lanelet cost-grid updates used for reachable goal filtering.
  void onCostGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    latest_cost_grid_ = *msg;
    has_latest_cost_grid_ = true;
    rebuildCostGridComponent();
  }

  // HH_260618: Treat Nav2 action success as the authoritative reached event.
  // Pose-distance is still used as a fallback when action status is delayed.
  void onNavStatus(const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
  {
    if (!has_active_released_goal_ || !msg) {
      return;
    }
    for (const auto & status : msg->status_list) {
      const double status_stamp_sec =
        static_cast<double>(status.goal_info.stamp.sec) +
        static_cast<double>(status.goal_info.stamp.nanosec) * 1e-9;
      const bool status_is_newer =
        active_released_sec_ <= 0.0 ||
        status_stamp_sec <= 0.0 ||
        status_stamp_sec + 0.5 >= active_released_sec_;
      if (!status_is_newer) {
        continue;
      }
      if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        active_goal_nav2_succeeded_ = true;
        markActiveGoalReached("nav2_status");
        if (sequential_goal_release_enable_) {
          releasePendingGoalIfReady();
        }
        return;
      }
    }
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
    handleSnappedGoal(out, px, py, std::sqrt(nearest.sq_dist), "avg");
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
    handleSnappedGoal(out_avg, px, py, std::sqrt(nearest.sq_dist), "ros");
  }

  void handleSnappedGoal(
    const avg_msgs::msg::PoseStamped & goal_pose,
    double input_x, double input_y, double snap_distance, const char * source_label)
  {
    if (!sequential_goal_release_enable_) {
      publishReleasedGoal(goal_pose, input_x, input_y, snap_distance, source_label, "direct");
      return;
    }

    releasePendingGoalIfReady();
    if (!has_active_released_goal_ || activeGoalReadyForNext()) {
      publishReleasedGoal(goal_pose, input_x, input_y, snap_distance, source_label, "released");
      return;
    }

    if (isSameGoal(goal_pose, active_released_goal_)) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 500,
        "goal_snapper(%s): skipped active duplicate queued goal", source_label);
      return;
    }
    enqueuePendingGoal(goal_pose, input_x, input_y, snap_distance, source_label);
  }

  void publishReleasedGoal(
    const avg_msgs::msg::PoseStamped & goal_pose,
    double input_x, double input_y, double snap_distance,
    const char * source_label, const char * release_reason)
  {
    pub_goal_->publish(goal_pose);
    publishRosGoal(goal_pose);
    active_released_goal_ = goal_pose;
    has_active_released_goal_ = true;
    active_goal_reached_ = false;
    active_goal_nav2_succeeded_ = false;
    active_goal_reached_sec_ = 0.0;
    active_released_sec_ = now().seconds();

    // HH_260316 Keep explicit runtime trace for path-failure diagnosis.
    // This confirms whether BT/NavigateToPose is using snapped goal coordinates.
    RCLCPP_INFO(
      get_logger(),
      "goal_snapper(%s): in=(%.2f, %.2f) out=(%.2f, %.2f) dist=%.2f mode=%s pending=%zu",
      source_label, input_x, input_y,
      goal_pose.pose.position.x, goal_pose.pose.position.y,
      snap_distance, release_reason, pending_goals_.size());
    publishAvgPlanning(goal_pose);
  }

  void enqueuePendingGoal(
    const avg_msgs::msg::PoseStamped & goal_pose,
    double input_x, double input_y, double snap_distance, const char * source_label)
  {
    if (sequential_goal_queue_policy_ == "drop_if_busy") {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "goal_snapper(%s): dropped goal while active goal is running in drop_if_busy mode",
        source_label);
      return;
    }

    if (sequential_goal_queue_policy_ == "replace_pending") {
      pending_goals_.clear();
    } else if (!pending_goals_.empty() && isSameGoal(goal_pose, pending_goals_.back())) {
      return;
    }

    while (static_cast<int>(pending_goals_.size()) >= sequential_goal_max_queue_size_) {
      pending_goals_.pop_front();
    }
    pending_goals_.push_back(goal_pose);
    RCLCPP_INFO(
      get_logger(),
      "goal_snapper(%s): queued pending goal in=(%.2f, %.2f) out=(%.2f, %.2f) dist=%.2f policy=%s pending=%zu",
      source_label, input_x, input_y,
      goal_pose.pose.position.x, goal_pose.pose.position.y,
      snap_distance, sequential_goal_queue_policy_.c_str(), pending_goals_.size());
  }

  void releasePendingGoalIfReady()
  {
    if (!sequential_goal_release_enable_ || pending_goals_.empty()) {
      return;
    }
    updateActiveGoalReachedFromPose();
    if (!activeGoalReadyForNext()) {
      return;
    }

    while (!pending_goals_.empty()) {
      const auto next_goal = pending_goals_.front();
      pending_goals_.pop_front();
      if (has_active_released_goal_ && isSameGoal(next_goal, active_released_goal_)) {
        continue;
      }
      publishReleasedGoal(next_goal, next_goal.pose.position.x, next_goal.pose.position.y, 0.0, "queue", "released_after_reached");
      return;
    }
  }

  // HH_260619 - Reissue the active snapped goal when the robot pose jumps > pose_jump_reissue_distance_m_.
  // Triggered by RViz "2D Pose Estimate" or localization re-init during an active NavigateToPose action.
  // Without reissue, Nav2 BT keeps the stale FollowPath context from the pre-jump pose and rotates in place.
  void maybeReissueActiveGoalOnPoseJump(double pose_jump_distance, double now_sec)
  {
    // HHL_260622 - Recheck reached state before reissuing. This prevents a manual
    // initialpose/P reset after arrival from resurrecting the previous snapped goal.
    updateActiveGoalReachedFromPose();
    if (
      !reissue_active_goal_on_pose_jump_ ||
      !has_active_released_goal_ ||
      active_goal_reached_ ||
      pose_jump_distance < pose_jump_reissue_distance_m_)
    {
      return;
    }

    const double min_interval_s = std::max(0.0, pose_jump_reissue_min_interval_s_);
    if (
      last_pose_jump_reissue_sec_ > 0.0 &&
      (now_sec - last_pose_jump_reissue_sec_) < min_interval_s)
    {
      return;
    }

    auto goal_pose = active_released_goal_;
    goal_pose.header.stamp = this->get_clock()->now();
    pub_goal_->publish(goal_pose);
    publishRosGoal(goal_pose);
    active_released_goal_ = goal_pose;
    active_released_sec_ = now_sec;
    active_goal_nav2_succeeded_ = false;
    last_pose_jump_reissue_sec_ = now_sec;

    RCLCPP_WARN(
      get_logger(),
      "goal_snapper: pose jump %.2fm detected; reissued active snapped goal (%.2f, %.2f) to rebuild Nav2 path",
      pose_jump_distance,
      goal_pose.pose.position.x, goal_pose.pose.position.y);
    publishAvgPlanning(goal_pose);
  }

  bool activeGoalReadyForNext()
  {
    if (!has_active_released_goal_) {
      return true;
    }
    if (!active_goal_reached_) {
      updateActiveGoalReachedFromPose();
    }
    if (!active_goal_reached_) {
      return false;
    }
    if (sequential_goal_require_nav2_terminal_ && !active_goal_nav2_succeeded_) {
      return false;
    }
    const double elapsed_s = now().seconds() - active_goal_reached_sec_;
    return elapsed_s >= std::max(0.0, sequential_goal_stop_hold_s_);
  }

  void updateActiveGoalReachedFromPose()
  {
    // HH_260618: Reached is sticky until a new active goal is released. Nav2 can
    // report SUCCEEDED while simulated/real pose drifts slightly outside the
    // distance threshold during controller settling; clearing the flag here
    // leaves queued goals stuck forever.
    if (active_goal_reached_ || !has_active_released_goal_ || !has_latest_current_pose_) {
      return;
    }
    const double dist = std::hypot(
      latest_current_pose_x_ - active_released_goal_.pose.position.x,
      latest_current_pose_y_ - active_released_goal_.pose.position.y);
    if (dist <= sequential_goal_reached_distance_m_) {
      markActiveGoalReached("pose_distance");
    }
  }

  void markActiveGoalReached(const char * source)
  {
    if (active_goal_reached_) {
      return;
    }
    active_goal_reached_ = true;
    active_goal_reached_sec_ = now().seconds();
    RCLCPP_INFO(
      get_logger(),
      "goal_snapper: active goal reached by %s; hold=%.2fs pending=%zu",
      source, sequential_goal_stop_hold_s_, pending_goals_.size());
  }

  bool isSameGoal(
    const avg_msgs::msg::PoseStamped & lhs,
    const avg_msgs::msg::PoseStamped & rhs) const
  {
    if (!lhs.header.frame_id.empty() && !rhs.header.frame_id.empty() &&
      lhs.header.frame_id != rhs.header.frame_id)
    {
      return false;
    }
    const double dxy = std::hypot(
      lhs.pose.position.x - rhs.pose.position.x,
      lhs.pose.position.y - rhs.pose.position.y);
    const double dz = std::fabs(lhs.pose.position.z - rhs.pose.position.z);
    return dxy <= sequential_goal_duplicate_xy_eps_m_ && dz <= duplicate_goal_z_eps_m_;
  }

  // Implements `snapGoal` behavior.
  bool snapGoal(
    const double px, const double py, const double pz,
    NearestResult & nearest, double & snapped_z)
  {
    nearest = findNearestCenterline(px, py, require_lanelet_containment_);
    bool used_uncontained_search = false;
    if (!nearest.valid && require_lanelet_containment_ && fallback_uncontained_) {
      used_uncontained_search = true;
      nearest = findNearestCenterline(px, py, false);
      if (nearest.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Goal is outside lanelet polygon; snapped to nearest centerline");
      }
    }
    if (used_uncontained_search && uncontained_global_snap_override_enable_) {
      const auto global_nearest = findNearestCenterline(px, py, false, true);
      if (global_nearest.valid) {
        const double selected_dist = nearest.valid ?
          std::sqrt(nearest.sq_dist) : std::numeric_limits<double>::infinity();
        const double global_dist = std::sqrt(global_nearest.sq_dist);
        const bool selected_is_too_far =
          !nearest.valid || selected_dist > uncontained_global_snap_max_distance_m_;
        const bool global_is_materially_better =
          selected_dist - global_dist >= uncontained_global_snap_min_improvement_m_;
        if (global_dist <= uncontained_global_snap_max_distance_m_ &&
          (selected_is_too_far || global_is_materially_better))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "goal_snapper: off-lane goal used bounded global nearest snap "
            "selected_dist=%.2f global_dist=%.2f",
            selected_dist, global_dist);
          nearest = global_nearest;
        }
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
    double x, double y, bool require_lanelet_containment,
    bool ignore_component_filters = false)
  {
    NearestResult best;
    const double max_sq = max_search_radius_ * max_search_radius_;

    const bool strict_component =
      !ignore_component_filters &&
      restrict_to_connected_lanelet_component_ && !allow_component_fallback_to_global_;
    const bool component_ready = !connected_lanelet_ids_.empty();
    if (strict_component && !component_ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: connected lanelet component is not ready yet; reject goal");
      return best;
    }

    for (const auto & ll : map_->laneletLayer) {
      if (!ignore_component_filters && restrict_to_connected_lanelet_component_ && component_ready) {
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
          if (!ignore_component_filters && !isPointInCostGridComponent(proj_x, proj_y)) {
            continue;
          }
          best.sq_dist = dist2;
          best.valid = true;
          const double proj_z = p0.z() + t * (p1.z() - p0.z());
          best.nearest_point = lanelet::Point3d(lanelet::InvalId, proj_x, proj_y, proj_z);
          best.heading = std::atan2(vy, vx);
        }
      }
    }

    if (!best.valid && !ignore_component_filters &&
      restrict_to_connected_lanelet_component_ && !component_ready &&
      allow_component_fallback_to_global_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: connected component unavailable; global fallback active");
    }

    return best;
  }

  // HH_260617: Rebuilds the connected free-space component from the current pose
  // in the same lanelet OccupancyGrid consumed by Nav2.
  void rebuildCostGridComponent()
  {
    if (!restrict_to_cost_grid_component_) {
      return;
    }
    has_cost_grid_component_ = false;
    cost_grid_component_mask_.clear();
    if (!has_latest_cost_grid_ || !has_latest_current_pose_) {
      return;
    }

    const auto & grid = latest_cost_grid_;
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    if (width <= 0 || height <= 0 || grid.info.resolution <= 0.0 ||
      grid.data.size() != static_cast<size_t>(width * height))
    {
      return;
    }

    int start_x = 0;
    int start_y = 0;
    if (!worldToCostGridCell(latest_current_pose_x_, latest_current_pose_y_, start_x, start_y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: current pose is outside cost grid; reachable goal filter is not ready");
      return;
    }

    const auto is_passable = [this, &grid](const int index) {
        const int value = static_cast<int>(grid.data[static_cast<size_t>(index)]);
        return value >= 0 && value < cost_grid_block_threshold_;
      };

    const int start_index = start_y * width + start_x;
    if (!is_passable(start_index)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "goal_snapper: current cost-grid cell is blocked cost=%d; reachable goal filter is not ready",
        static_cast<int>(grid.data[static_cast<size_t>(start_index)]));
      return;
    }

    cost_grid_component_mask_.assign(static_cast<size_t>(width * height), 0U);
    std::deque<int> queue;
    queue.push_back(start_index);
    cost_grid_component_mask_[static_cast<size_t>(start_index)] = 1U;

    while (!queue.empty()) {
      const int index = queue.front();
      queue.pop_front();
      const int x = index % width;
      const int y = index / width;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const int next_index = ny * width + nx;
          const auto next_offset = static_cast<size_t>(next_index);
          if (cost_grid_component_mask_[next_offset] != 0U || !is_passable(next_index)) {
            continue;
          }
          cost_grid_component_mask_[next_offset] = 1U;
          queue.push_back(next_index);
        }
      }
    }
    has_cost_grid_component_ = true;
  }

  bool worldToCostGridCell(double x, double y, int & cell_x, int & cell_y) const
  {
    if (!has_latest_cost_grid_) {
      return false;
    }
    const auto & info = latest_cost_grid_.info;
    if (info.resolution <= 0.0) {
      return false;
    }
    cell_x = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
    cell_y = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
    return cell_x >= 0 && cell_y >= 0 &&
      cell_x < static_cast<int>(info.width) && cell_y < static_cast<int>(info.height);
  }

  bool isPointInCostGridComponent(double x, double y)
  {
    if (!restrict_to_cost_grid_component_) {
      return true;
    }
    if (!has_cost_grid_component_) {
      if (allow_cost_grid_component_fallback_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "goal_snapper: cost-grid component unavailable; using lanelet-only snap filter");
        return true;
      }
      return false;
    }
    int cell_x = 0;
    int cell_y = 0;
    if (!worldToCostGridCell(x, y, cell_x, cell_y)) {
      return false;
    }
    const int width = static_cast<int>(latest_cost_grid_.info.width);
    const auto index = static_cast<size_t>(cell_y * width + cell_x);
    return index < cost_grid_component_mask_.size() &&
      cost_grid_component_mask_[index] != 0U;
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

  // HH_260618 Sequential release state for non-preemptive waypoint behavior.
  bool sequential_goal_release_enable_{true};
  std::string sequential_goal_queue_policy_{"append"};
  double sequential_goal_reached_distance_m_{0.8};
  double sequential_goal_stop_hold_s_{0.5};
  double sequential_goal_duplicate_xy_eps_m_{0.20};
  int sequential_goal_max_queue_size_{10};
  std::string sequential_goal_status_topic_{"/planning/navigate_to_pose/_action/status"};
  bool sequential_goal_require_nav2_terminal_{true};
  // HH_260619 - Pose-jump reissue state tracks last known pose and last reissue timestamp.
  bool reissue_active_goal_on_pose_jump_{true};
  double pose_jump_reissue_distance_m_{1.5};
  double pose_jump_reissue_min_interval_s_{1.0};
  std::deque<avg_msgs::msg::PoseStamped> pending_goals_;
  avg_msgs::msg::PoseStamped active_released_goal_;
  bool has_active_released_goal_{false};
  bool active_goal_reached_{false};
  bool active_goal_nav2_succeeded_{false};
  double active_goal_reached_sec_{0.0};
  double active_released_sec_{0.0};
  // HH_260619 - Previous pose checkpoint for per-callback jump distance measurement.
  bool has_pose_jump_check_pose_{false};
  double last_pose_jump_check_x_{0.0};
  double last_pose_jump_check_y_{0.0};
  double last_pose_jump_reissue_sec_{0.0};

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

  // HH_260617 Cost-grid component restriction state.
  bool restrict_to_cost_grid_component_{true};
  bool allow_cost_grid_component_fallback_{false};
  bool uncontained_global_snap_override_enable_{true};
  double uncontained_global_snap_max_distance_m_{15.0};
  double uncontained_global_snap_min_improvement_m_{5.0};
  std::string cost_grid_topic_{"/map/cost_grid/lanelet"};
  int cost_grid_block_threshold_{100};
  nav_msgs::msg::OccupancyGrid latest_cost_grid_;
  bool has_latest_cost_grid_{false};
  bool has_latest_current_pose_{false};
  double latest_current_pose_x_{0.0};
  double latest_current_pose_y_{0.0};
  std::vector<uint8_t> cost_grid_component_mask_;
  bool has_cost_grid_component_{false};

  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_ros_;
  rclcpp::Publisher<AvgPlanningMsgs>::SharedPtr pub_avg_planning_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_ros_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_current_pose_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_cost_grid_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_nav_status_;
  rclcpp::TimerBase::SharedPtr queue_timer_;
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
