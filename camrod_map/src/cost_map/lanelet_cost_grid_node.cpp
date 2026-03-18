#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <avg_msgs/msg/occupancy_grid.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <avg_msgs/msg/avg_map_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

#include "camrod_map/custom_regulatory_elements.hpp"  // HH_260101 register speed_bump
#include <avg_msgs/msg/point.hpp>
#include <avg_msgs/msg/set_parameters_result.hpp>

// HH_251231 Simple lanelet-based OccupancyGrid publisher for Nav2 costmap layer.
// Marks lanelet interior as low cost (0), outside as lethal (100).
// No LiDAR dependency; intended for localization/keep-out visualization.
class LaneletCostGridNode : public rclcpp::Node
{
public:
  // HH_260112 Use short node name; namespace applies the module prefix.
  LaneletCostGridNode() : Node("lanelet_cost_grid")
  {
    // HH_260128 require explicit map_path (YAML/launch); empty -> error.
    map_path_ = declare_parameter<std::string>("map_path", "");
    frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    resolution_ = declare_parameter<double>("resolution", 0.5);
    window_width_ = declare_parameter<int>("width", 400);   // HH_260102 window size around robot/path (cells)
    window_height_ = declare_parameter<int>("height", 400);
    origin_x_ = declare_parameter<double>("origin_x", -50.0);   // HH_251231 initial origin
    origin_y_ = declare_parameter<double>("origin_y", -50.0);
    centerline_half_width_ = declare_parameter<double>("centerline_half_width", 1.5);  // HH_260101 corridor half width (m)
    // HH_260316-00:00 Keep centerline corridor inside lanelet polygon by default.
    // This avoids planner-base strips leaking outside lane bounds at sharp corners.
    centerline_clip_to_lanelet_ = declare_parameter<bool>("centerline_clip_to_lanelet", true);
    // HH_260316-00:00 Optional lanelet interior base cost for centerline mode.
    // -1 disables lanelet fill (centerline strip only).
    // >=0 enables robust traversable connectivity while keeping centerline as lowest cost.
    centerline_lanelet_fill_value_ = declare_parameter<int>("centerline_lanelet_fill_value", -1);
    origin_lat_ = declare_parameter<double>("offset_lat", 0.0);   // HH_260101 map origin for projection
    origin_lon_ = declare_parameter<double>("offset_lon", 0.0);
    origin_alt_ = declare_parameter<double>("offset_alt", 0.0);
    // HH_260316-00:00 Keep lanelet projector selectable.
    // Use LocalCartesian by default so this node matches:
    // - lanelet2_map_loader
    // - planning goal/centerline snappers
    // and avoids cross-node XY mismatches.
    projector_type_ = declare_parameter<std::string>("projector_type", "local_cartesian");
    free_value_ = declare_parameter<int>("free_value", 0);
    lethal_value_ = declare_parameter<int>("lethal_value", 100);
    // 2026-02-02 11:55: Cost mode controls how the grid is rasterized.
    //   centerline = low-cost corridor around lanelet centerlines (default)
    //   bounds     = high-cost thin strips along left/right boundaries only
    //   path       = low-cost corridor along path, everything else high
    cost_mode_ = declare_parameter<std::string>("cost_mode", "centerline");
    boundary_half_width_ = declare_parameter<double>("boundary_half_width", 0.2);
    // 2026-02-26: Optional boundary overlay when cost_mode == lanelet.
    // -1 disables boundary overlay, otherwise [0..100] is painted on lane boundaries.
    lanelet_boundary_value_ = declare_parameter<int>("lanelet_boundary_value", -1);
    // 2026-02-27: Clip boundary strips to lanelet interior to prevent outward bleed.
    boundary_clip_to_lanelet_ = declare_parameter<bool>("boundary_clip_to_lanelet", true);
    // 2026-02-27: Debug counters for boundary rasterization quality.
    debug_boundary_stats_ = declare_parameter<bool>("debug_boundary_stats", false);
    // 2026-02-27: Optional diagnostics for known-cost cells outside lanelet polygons.
    debug_coverage_stats_ = declare_parameter<bool>("debug_coverage_stats", false);
    debug_coverage_stride_ = declare_parameter<int>("debug_coverage_stride", 3);
    debug_coverage_min_value_ = declare_parameter<int>("debug_coverage_min_value", 0);
    outside_value_ = declare_parameter<int>("outside_value", -1);
    direction_penalty_ = declare_parameter<int>("direction_penalty", 80);  // HH_260101 penalty for opposite heading (raise to make opposite lane expensive)
    backward_penalty_ = declare_parameter<int>("backward_penalty", 60);  // HH_260102 penalize behind-robot cells to discourage reverse
    gradient_range_ = declare_parameter<double>("gradient_range", 30.0);  // HH_260101 decay distance for cost (m)
    // 2026-02-11: For static centerline corridor maps, keep in-lane cost flat by default.
    centerline_use_distance_gradient_ = declare_parameter<bool>("centerline_use_distance_gradient", false);
    // 2026-02-11: Path-mode grids can be rendered only inside lanelet areas (no rectangular background fill).
    path_use_lanelet_mask_ = declare_parameter<bool>("path_use_lanelet_mask", true);
    // 2026-02-11: Base cost inside lanelet area before path strip overlay in path mode.
    path_lane_base_value_ = declare_parameter<int>("path_lane_base_value", 100);
    // 2026-02-24: Local path mode can mask only lanelets that the path traverses.
    path_lanelet_only_ = declare_parameter<bool>("path_lanelet_only", false);
    path_lanelet_match_max_dist_ = declare_parameter<double>("path_lanelet_match_max_dist", 5.0);
    // HH_260305-00:00 Optional nearest-lanelet fallback for sparse/off-lane path samples.
    // Disable for strict path-only masking to prevent remote/stray lanelet patches.
    path_lanelet_allow_nearest_fallback_ =
      declare_parameter<bool>("path_lanelet_allow_nearest_fallback", true);
    // 2026-02-24: Densify path->lanelet matching to avoid missing lanelets on curved/sparse paths.
    path_lanelet_sample_step_ = declare_parameter<double>("path_lanelet_sample_step", 0.4);
    path_clip_to_lanelet_ = declare_parameter<bool>("path_clip_to_lanelet", true);
    // 2026-03-04: Optional global-path gradient by current vehicle position.
    // Keep disabled for local-path grids so only path-distance matters there.
    path_use_pose_distance_gradient_ =
      declare_parameter<bool>("path_use_pose_distance_gradient", false);
    // HH_260313-00:00 Path strip can keep centerline as lowest-cost band.
    path_use_lateral_gradient_ = declare_parameter<bool>("path_use_lateral_gradient", true);
    // HH_260313-00:00 Path-strip blend weights (pose-distance vs lateral-distance terms).
    path_pose_cost_weight_ = std::max(0.0, declare_parameter<double>("path_pose_cost_weight", 0.7));
    path_lateral_cost_weight_ = std::max(
      0.0, declare_parameter<double>("path_lateral_cost_weight", 0.3));
    grid_yaw_ = declare_parameter<double>("grid_yaw", 0.0);  // HH_260103 manual yaw (rad) for OccupancyGrid orientation
    use_path_bbox_ = declare_parameter<bool>("use_path_bbox", false);  // HH_260103 false -> robot-centered window, true -> path bbox window
    lock_window_ = declare_parameter<bool>("lock_window", false);  // HH_260126 keep grid window fixed (use origin_x/y + width/height)
    // 2026-01-30: Allow fixed window to follow map bounds without hardcoding width/height.
    use_map_bbox_ = declare_parameter<bool>("use_map_bbox", false);
    ignore_invalid_map_ = declare_parameter<bool>("ignore_invalid_map", false);  // HH_260127 allow loading maps with OSM validation warnings
    // HH_260109 default to fused localization pose for lanelet-guided cost grid.
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/planning/global_path");  // HH_260103 focus cost along planned path
    // 2026-02-25: Optional fallback path input (e.g. local path primary + global path fallback).
    path_fallback_topic_ = declare_parameter<std::string>("path_fallback_topic", "");
    // 2026-02-26: Optional goal topic to clear stale path visualization immediately on new goal.
    goal_topic_ = declare_parameter<std::string>("goal_topic", "");
    // 2026-02-26: Ignore fallback path while primary path is fresh.
    primary_path_timeout_sec_ = declare_parameter<double>("primary_path_timeout_sec", 1.0);
    // 2026-02-26: Ignore fallback path briefly after goal update to avoid old-path flash.
    goal_fallback_holdoff_sec_ = declare_parameter<double>("goal_fallback_holdoff_sec", 0.6);
    // 2026-02-26: Optional stale path auto-clear (<=0 disables).
    stale_path_timeout_sec_ = declare_parameter<double>("stale_path_timeout_sec", 0.0);
    // HH_260317-00:00 Guard against stale route reuse after a new goal is received.
    // When enabled, path messages older than latest goal timestamp are ignored.
    drop_stale_path_after_goal_ =
      declare_parameter<bool>("drop_stale_path_after_goal", true);
    // HH_260317-00:00 Allow small timestamp skew between goal/path publishers.
    path_goal_stamp_slack_sec_ =
      declare_parameter<double>("path_goal_stamp_slack_sec", 0.10);
    // HH_260317-00:00 Prefer geometric freshness check:
    // accept only paths that terminate near the latest goal.
    // This is more robust than header-stamp-only checks across mixed publishers.
    fresh_path_goal_match_tolerance_m_ =
      declare_parameter<double>("fresh_path_goal_match_tolerance_m", 2.0);
    // 2026-02-27: Keep previous path on empty transient updates to avoid marker flicker.
    ignore_empty_path_ = declare_parameter<bool>("ignore_empty_path", true);
    // 2026-02-27: Keep previous path until a new one arrives on goal update.
    clear_path_on_goal_ = declare_parameter<bool>("clear_path_on_goal", false);
    // 2026-02-27: Rebuild throttling controls to prevent path/cost lag under high CPU load.
    rebuild_on_pose_ = declare_parameter<bool>("rebuild_on_pose", true);
    rebuild_on_path_ = declare_parameter<bool>("rebuild_on_path", true);
    min_rebuild_period_sec_ = declare_parameter<double>("min_rebuild_period_sec", 0.0);
    // HH_260305-00:00 Allow path-mode grid baseline build (lanelet mask + pose gradient)
    // even when no valid path has been received yet.
    allow_build_without_path_ = declare_parameter<bool>("allow_build_without_path", false);
    // HH_260305-00:00 Debug counters for rebuild trigger/skip reasons.
    debug_rebuild_stats_ = declare_parameter<bool>("debug_rebuild_stats", false);
    debug_build_timing_ = declare_parameter<bool>("debug_build_timing", false);
    // HH_260305-00:00 Inside-hit threshold for cellMostlyInsidePolygon (9-point sampling).
    // Base(default) threshold.
    cell_inside_min_hits_ = std::clamp(
      static_cast<int>(declare_parameter<int>("cell_inside_min_hits", 5)), 1, 9);
    // HH_260305-00:00 Path/centerline strip threshold.
    // Lower than boundary threshold to keep curved segments contiguous.
    cell_inside_min_hits_path_ = std::clamp(
      static_cast<int>(declare_parameter<int>("cell_inside_min_hits_path", cell_inside_min_hits_)),
      1, 9);
    // HH_260305-00:00 Boundary strip threshold.
    // Keep stricter to prevent outward bleed across sharp corners.
    cell_inside_min_hits_boundary_ = std::clamp(
      static_cast<int>(
        declare_parameter<int>("cell_inside_min_hits_boundary", cell_inside_min_hits_)),
      1, 9);
    // 2026-02-27: Allow latched path consumption when path publishers are transient_local.
    path_qos_transient_local_ = declare_parameter<bool>("path_qos_transient_local", false);
    republish_period_ = declare_parameter<double>("republish_period", 1.0);  // HH_260103 RViz toggle refresh
    // HH_260311-00:00 Optional timer-driven rebuild.
    // Enables deterministic realtime gradient refresh even when path/pose callbacks are sparse.
    rebuild_on_timer_ = declare_parameter<bool>("rebuild_on_timer", false);
    output_topic_ = declare_parameter<std::string>("output_topic", "/map/cost_grid/lanelet");  // HH_260123 allow multiple cost grids
    publish_map_diagnostic_ = declare_parameter<bool>("publish_map_diagnostic", false);
    map_diagnostic_topic_ = declare_parameter<std::string>("map_diagnostic_topic", "/map/diagnostic");

    // HH_260109 Publish lanelet cost grid under /map prefix.
    grid_pub_ = create_publisher<avg_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local());
    if (publish_map_diagnostic_) {
      avg_map_pub_ = create_publisher<avg_msgs::msg::AvgMapMsgs>(
        map_diagnostic_topic_, rclcpp::QoS(10));
    }
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&LaneletCostGridNode::onParamChange, this, std::placeholders::_1));
    updateRepublishTimer();

    if (map_path_.empty() || !std::filesystem::exists(map_path_)) {
      RCLCPP_ERROR(get_logger(), "map_path invalid or missing: '%s'", map_path_.c_str());
      return;
    }

    try {
      lanelet::ErrorMessages errors;
      const lanelet::Origin origin(lanelet::GPSPoint{origin_lat_, origin_lon_, origin_alt_});
      const std::string projector_norm = toLower(projector_type_);
      if (projector_norm == "utm") {
        // HH_260316-00:00 Optional UTM mode for compatibility tests.
        lanelet::projection::UtmProjector projector(origin);
        map_ = lanelet::load(map_path_, projector, &errors);
      } else {
        // HH_260316-00:00 Default projector aligned with map/snapper nodes.
        lanelet::projection::LocalCartesianProjector projector(origin);
        map_ = lanelet::load(map_path_, projector, &errors);
      }
      if (!errors.empty()) {
        for (const auto & msg : errors) {
          if (ignore_invalid_map_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map load warning: %s", msg.c_str());
          } else {
            RCLCPP_ERROR(get_logger(), "map load error: %s", msg.c_str());
          }
        }
        if (!ignore_invalid_map_ && !errors.empty()) {
          return;
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "failed to load lanelet map: %s", e.what());
      return;
    }
    computeMapBounds();
    buildAllLaneletPolygons();
    RCLCPP_INFO(
      get_logger(), "lanelet_cost_grid projector=%s map=%s",
      projector_type_.c_str(), map_path_.c_str());
    if (!requiresPoseForBuild()) {
      buildGrid();
      has_built_grid_ = !last_grid_.data.empty();
      last_build_time_ = now();
    }

    // HH_251231 subscribe to robot pose and build grid around robot
    using std::placeholders::_1;
    // HH_260305-00:00 Use reliable/latest-only pose QoS for deterministic rebuild timing.
    // Best-effort drops can leave local/global path cost grids stale at old poses.
    pose_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&LaneletCostGridNode::onPose, this, _1));
    auto path_qos = rclcpp::QoS(1).reliable();
    if (path_qos_transient_local_) {
      path_qos.transient_local();
    }
    path_sub_ = create_subscription<avg_msgs::msg::Path>(
      path_topic_, path_qos,
      std::bind(&LaneletCostGridNode::onPathPrimary, this, _1));
    if (!path_fallback_topic_.empty() && path_fallback_topic_ != path_topic_) {
      path_fallback_sub_ = create_subscription<avg_msgs::msg::Path>(
        path_fallback_topic_, path_qos,
        std::bind(&LaneletCostGridNode::onPathFallback, this, _1));
    }
    if (!goal_topic_.empty()) {
      goal_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
        goal_topic_, rclcpp::QoS(10),
        std::bind(&LaneletCostGridNode::onGoal, this, _1));
    }
    RCLCPP_DEBUG(get_logger(), "waiting pose on %s to build cost grid", pose_topic_.c_str());

    // HH_260125 TF listener to align incoming poses/paths to map_frame_ if frame_id differs.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Handles the `onRepublishTimer` callback.
  void onRepublishTimer()
  {
    if (debug_rebuild_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rebuild stats: pose_cb=%ld path_cb=%ld build_ok=%ld skip_no_path=%ld skip_throttle=%ld",
        pose_cb_count_, path_cb_count_, build_ok_count_,
        skip_no_path_count_, skip_throttle_count_);
    }
    if (stale_path_timeout_sec_ > 0.0 && path_received_ && last_any_path_rx_.nanoseconds() > 0) {
      const double dt = (now() - last_any_path_rx_).seconds();
      if (dt > stale_path_timeout_sec_) {
        path_received_ = false;
        path_.poses.clear();
        invalidatePathLaneletCache();
        // HH_260305-00:00 Path-mode baseline is optional.
        // - allow_build_without_path=true : rebuild lanelet-mask baseline immediately.
        // - false                        : clear stale path grid.
        if (cost_mode_ == "path" && allow_build_without_path_ && has_pose_) {
          requestBuild(true);
        } else {
          clearPublishedGrid();
        }
      }
    }
    if (rebuild_on_timer_) {
      requestBuild(false);
    }
    if (!last_grid_.data.empty()) {
      last_grid_.header.stamp = now();
      grid_pub_->publish(last_grid_);
      publishAvgMapGrid(last_grid_, "lanelet cost grid republished");
    }
  }

private:
  // Implements `toLower` behavior.
  static std::string toLower(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return value;
  }

  // Computes `MapBounds` values.
  void computeMapBounds()
  {
    if (!map_) {
      return;
    }
    bool first = true;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    for (const auto & pt : map_->pointLayer) {
      const double x = pt.x();
      const double y = pt.y();
      if (first) {
        min_x = max_x = x;
        min_y = max_y = y;
        first = false;
        continue;
      }
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
    if (!first) {
      map_min_x_ = min_x;
      map_min_y_ = min_y;
      map_max_x_ = max_x;
      map_max_y_ = max_y;
      map_bounds_valid_ = true;
    }
  }

  // Handles the `onPose` callback.
  void onPose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    ++pose_cb_count_;
    avg_msgs::msg::PoseStamped pose_in_map;
    if (!transformToMap(*msg, pose_in_map)) {
      return;
    }
    current_pose_ = pose_in_map;
    has_pose_ = true;
    // HH_260305-00:00 In path mode, optionally allow baseline lanelet build without path.
    if (
      cost_mode_ == "path" &&
      (!path_received_ || path_.poses.size() < 2) &&
      !allow_build_without_path_)
    {
      return;
    }
    // Always build once after first valid pose so static grids initialize.
    if (!has_built_grid_) {
      requestBuild(true);
      return;
    }
    if (rebuild_on_pose_) {
      requestBuild(false);
    }
  }

  // Handles the `onPathPrimary` callback.
  void onPathPrimary(const avg_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (onPathCommon(msg)) {
      last_primary_path_rx_ = now();
    }
  }

  // Handles the `onPathFallback` callback.
  void onPathFallback(const avg_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (fallback_holdoff_until_.nanoseconds() > 0 && now() < fallback_holdoff_until_) {
      return;
    }
    // Fallback keeps the grid informative when the primary path topic is idle.
    if (last_primary_path_rx_.nanoseconds() > 0) {
      const double dt = (now() - last_primary_path_rx_).seconds();
      if (dt < primary_path_timeout_sec_) {
        return;
      }
    }
    (void)onPathCommon(msg);
  }

  // Handles the `onPathCommon` callback.
  bool onPathCommon(const avg_msgs::msg::Path::ConstSharedPtr msg)
  {
    ++path_cb_count_;
    if (msg->poses.size() < 2) {
      if (ignore_empty_path_) {
        return false;
      }
      path_received_ = false;
      path_.poses.clear();
      invalidatePathLaneletCache();
      last_any_path_rx_ = now();
      if (cost_mode_ == "path" && allow_build_without_path_ && has_pose_) {
        requestBuild(true);
      } else {
        clearPublishedGrid();
      }
      return true;
    }
    avg_msgs::msg::Path path_map;
    path_map.header = msg->header;
    path_map.header.frame_id = frame_id_;
    path_map.poses.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      avg_msgs::msg::PoseStamped ps_map;
      if (!transformToMap(ps, ps_map)) {
        continue;
      }
      path_map.poses.push_back(ps_map);
    }
    if (path_map.poses.size() < 2) {
      if (ignore_empty_path_) {
        return false;
      }
      path_received_ = false;
      path_.poses.clear();
      invalidatePathLaneletCache();
      last_any_path_rx_ = now();
      if (cost_mode_ == "path" && allow_build_without_path_ && has_pose_) {
        requestBuild(true);
      } else {
        clearPublishedGrid();
      }
      return true;
    }
    if (drop_stale_path_after_goal_ && waiting_fresh_path_after_goal_) {
      bool accepted_as_fresh = false;
      // HH_260317-00:00 Primary freshness gate:
      // the latest path endpoint must match the latest goal pose.
      if (has_latest_goal_pose_) {
        const auto & end = path_map.poses.back().pose.position;
        const auto & goal = latest_goal_pose_.pose.position;
        const double dist_to_goal = std::hypot(end.x - goal.x, end.y - goal.y);
        if (dist_to_goal <= std::max(0.0, fresh_path_goal_match_tolerance_m_)) {
          accepted_as_fresh = true;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ignore stale/mismatched path after goal update (end_goal_dist=%.2f tol=%.2f)",
            dist_to_goal, std::max(0.0, fresh_path_goal_match_tolerance_m_));
          return false;
        }
      }
      // HH_260317-00:00 Secondary fallback:
      // use timestamp ordering only when goal geometry is not available.
      if (
        !accepted_as_fresh &&
        last_goal_rx_.nanoseconds() > 0 &&
        (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0))
      {
        const rclcpp::Time path_stamp(msg->header.stamp);
        const auto slack = rclcpp::Duration::from_seconds(
          std::max(0.0, path_goal_stamp_slack_sec_));
        if (path_stamp + slack < last_goal_rx_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ignore stale path older than latest goal (path=%.3f goal=%.3f)",
            path_stamp.seconds(), last_goal_rx_.seconds());
          return false;
        }
      }
    }
    path_ = path_map;
    path_received_ = true;
    waiting_fresh_path_after_goal_ = false;
    invalidatePathLaneletCache();
    last_any_path_rx_ = now();
    if (has_pose_ && rebuild_on_path_) {
      requestBuild(true);
    }
    return true;
  }

  // Handles the `onGoal` callback.
  void onGoal(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (msg && (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)) {
      last_goal_rx_ = rclcpp::Time(msg->header.stamp);
    } else {
      last_goal_rx_ = now();
    }
    if (msg) {
      avg_msgs::msg::PoseStamped goal_map;
      if (transformToMap(*msg, goal_map)) {
        latest_goal_pose_ = goal_map;
        has_latest_goal_pose_ = true;
      } else {
        has_latest_goal_pose_ = false;
      }
    }
    waiting_fresh_path_after_goal_ = true;
    if (clear_path_on_goal_) {
      // Optional strict behavior: clear old path immediately on goal update.
      path_received_ = false;
      path_.poses.clear();
      invalidatePathLaneletCache();
      last_any_path_rx_ = now();
      if (cost_mode_ == "path" && allow_build_without_path_ && has_pose_) {
        requestBuild(true);
      } else {
        clearPublishedGrid();
      }
    }
    if (goal_fallback_holdoff_sec_ > 0.0) {
      fallback_holdoff_until_ = now() + rclcpp::Duration::from_seconds(goal_fallback_holdoff_sec_);
    }

    // HH_260306-00:00 Rebuild immediately on goal updates so path-cost markers
    // react even before the next path message arrives.
    if (has_pose_) {
      requestBuild(true);
    }
  }

  // Handles the `onParamChange` callback.
  avg_msgs::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "centerline_half_width") {
        centerline_half_width_ = p.as_double();
      } else if (p.get_name() == "resolution") {
        resolution_ = p.as_double();
      } else if (p.get_name() == "width") {
        window_width_ = p.as_int();
      } else if (p.get_name() == "height") {
        window_height_ = p.as_int();
      } else if (p.get_name() == "free_value") {
        free_value_ = p.as_int();
      } else if (p.get_name() == "lethal_value") {
        lethal_value_ = p.as_int();
      } else if (p.get_name() == "gradient_range") {
        gradient_range_ = p.as_double();
      } else if (p.get_name() == "centerline_use_distance_gradient") {
        centerline_use_distance_gradient_ = p.as_bool();
      } else if (p.get_name() == "centerline_clip_to_lanelet") {
        centerline_clip_to_lanelet_ = p.as_bool();
      } else if (p.get_name() == "centerline_lanelet_fill_value") {
        centerline_lanelet_fill_value_ = p.as_int();
      } else if (p.get_name() == "path_use_lanelet_mask") {
        path_use_lanelet_mask_ = p.as_bool();
      } else if (p.get_name() == "path_lane_base_value") {
        path_lane_base_value_ = p.as_int();
      } else if (p.get_name() == "path_lanelet_only") {
        path_lanelet_only_ = p.as_bool();
        invalidatePathLaneletCache();
      } else if (p.get_name() == "path_lanelet_match_max_dist") {
        path_lanelet_match_max_dist_ = p.as_double();
        invalidatePathLaneletCache();
      } else if (p.get_name() == "path_lanelet_allow_nearest_fallback") {
        path_lanelet_allow_nearest_fallback_ = p.as_bool();
        invalidatePathLaneletCache();
      } else if (p.get_name() == "path_lanelet_sample_step") {
        path_lanelet_sample_step_ = p.as_double();
        invalidatePathLaneletCache();
      } else if (p.get_name() == "path_clip_to_lanelet") {
        path_clip_to_lanelet_ = p.as_bool();
        invalidatePathLaneletCache();
      } else if (p.get_name() == "path_use_pose_distance_gradient") {
        path_use_pose_distance_gradient_ = p.as_bool();
      } else if (p.get_name() == "path_use_lateral_gradient") {
        path_use_lateral_gradient_ = p.as_bool();
      } else if (p.get_name() == "path_pose_cost_weight") {
        path_pose_cost_weight_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "path_lateral_cost_weight") {
        path_lateral_cost_weight_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "ignore_empty_path") {
        ignore_empty_path_ = p.as_bool();
      } else if (p.get_name() == "path_use_lanelet_mask") {
        path_use_lanelet_mask_ = p.as_bool();
        invalidatePathLaneletCache();
      } else if (p.get_name() == "clear_path_on_goal") {
        clear_path_on_goal_ = p.as_bool();
      } else if (p.get_name() == "rebuild_on_pose") {
        rebuild_on_pose_ = p.as_bool();
      } else if (p.get_name() == "rebuild_on_path") {
        rebuild_on_path_ = p.as_bool();
      } else if (p.get_name() == "min_rebuild_period_sec") {
        min_rebuild_period_sec_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "allow_build_without_path") {
        allow_build_without_path_ = p.as_bool();
      } else if (p.get_name() == "drop_stale_path_after_goal") {
        drop_stale_path_after_goal_ = p.as_bool();
      } else if (p.get_name() == "path_goal_stamp_slack_sec") {
        path_goal_stamp_slack_sec_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "fresh_path_goal_match_tolerance_m") {
        fresh_path_goal_match_tolerance_m_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "backward_penalty") {
        backward_penalty_ = p.as_int();
      } else if (p.get_name() == "direction_penalty") {
        direction_penalty_ = p.as_int();
      } else if (p.get_name() == "debug_build_timing") {
        debug_build_timing_ = p.as_bool();
      } else if (p.get_name() == "grid_yaw") {
        grid_yaw_ = p.as_double();
      } else if (p.get_name() == "cost_mode") {
        cost_mode_ = p.as_string();
      } else if (p.get_name() == "boundary_half_width") {
        boundary_half_width_ = p.as_double();
      } else if (p.get_name() == "lanelet_boundary_value") {
        lanelet_boundary_value_ = p.as_int();
      } else if (p.get_name() == "boundary_clip_to_lanelet") {
        boundary_clip_to_lanelet_ = p.as_bool();
      } else if (p.get_name() == "debug_boundary_stats") {
        debug_boundary_stats_ = p.as_bool();
      } else if (p.get_name() == "debug_coverage_stats") {
        debug_coverage_stats_ = p.as_bool();
      } else if (p.get_name() == "debug_coverage_stride") {
        debug_coverage_stride_ = std::max(1, static_cast<int>(p.as_int()));
      } else if (p.get_name() == "debug_coverage_min_value") {
        debug_coverage_min_value_ = static_cast<int>(p.as_int());
      } else if (p.get_name() == "outside_value") {
        outside_value_ = p.as_int();
      } else if (p.get_name() == "cell_inside_min_hits") {
        cell_inside_min_hits_ = std::clamp(static_cast<int>(p.as_int()), 1, 9);
      } else if (p.get_name() == "cell_inside_min_hits_path") {
        cell_inside_min_hits_path_ = std::clamp(static_cast<int>(p.as_int()), 1, 9);
      } else if (p.get_name() == "cell_inside_min_hits_boundary") {
        cell_inside_min_hits_boundary_ = std::clamp(static_cast<int>(p.as_int()), 1, 9);
      } else if (p.get_name() == "use_map_bbox") {
        use_map_bbox_ = p.as_bool();
      } else if (p.get_name() == "republish_period") {
        republish_period_ = std::max(0.0, p.as_double());
      } else if (p.get_name() == "rebuild_on_timer") {
        rebuild_on_timer_ = p.as_bool();
      }
    }
    updateRepublishTimer();
    // regenerate grid if pose already exists
    if (has_pose_) {
      requestBuild(true);
    }
    avg_msgs::msg::SetParametersResult res;
    res.successful = true;
    res.reason = "updated";
    return res;
  }

  // Implements `buildGrid` behavior.
  void buildGrid()
  {
    if (!map_) return;
    if (!has_pose_ && requiresPoseForBuild()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "pose not received yet; skip grid publish");
      return;
    }
    avg_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.info.resolution = resolution_;

    double yaw = grid_yaw_;
    // 2026-02-26: Rasterizer is currently axis-aligned in map frame.
    // Non-zero grid yaw rotates metadata only and causes visual mismatch.
    if (std::abs(yaw) > 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "grid_yaw=%.4f is not supported by rasterizer; forcing 0.0 to avoid map/cost mismatch", yaw);
      yaw = 0.0;
    }
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    // HH_260114 If path exists, use path bounding box to fix grid size (avoid flicker).
    double min_x = current_pose_.pose.position.x;
    double max_x = current_pose_.pose.position.x;
    double min_y = current_pose_.pose.position.y;
    double max_y = current_pose_.pose.position.y;
    if (lock_window_) {
      if (use_map_bbox_ && map_bounds_valid_) {
        // 2026-01-30: Use full map bounds with margin for a stable global window.
        const double margin = computeRasterPadding();
        min_x = map_min_x_ - margin;
        min_y = map_min_y_ - margin;
        max_x = map_max_x_ + margin;
        max_y = map_max_y_ + margin;
      } else {
        // HH_260126 Fixed window anchored at configured origin/size (map frame).
        min_x = origin_x_;
        min_y = origin_y_;
        max_x = origin_x_ + window_width_ * resolution_;
        max_y = origin_y_ + window_height_ * resolution_;
      }
    } else if (use_path_bbox_ && path_received_ && path_.poses.size() > 1) {
      // HH_260125 Fixed window: derive solely from path, not current pose, to avoid drift.
      min_x = path_.poses.front().pose.position.x;
      max_x = min_x;
      min_y = path_.poses.front().pose.position.y;
      max_y = min_y;
      for (const auto & ps : path_.poses) {
        min_x = std::min(min_x, ps.pose.position.x);
        max_x = std::max(max_x, ps.pose.position.x);
        min_y = std::min(min_y, ps.pose.position.y);
        max_y = std::max(max_y, ps.pose.position.y);
      }
      const double margin = computeRasterPadding();
      min_x -= margin;
      max_x += margin;
      min_y -= margin;
      max_y += margin;
    } else {
      // HH_260114 If no path, keep robot-centered window.
      const double win_half_x = 0.5 * window_width_ * resolution_;
      const double win_half_y = 0.5 * window_height_ * resolution_;
      min_x = current_pose_.pose.position.x - win_half_x;
      max_x = current_pose_.pose.position.x + win_half_x;
      min_y = current_pose_.pose.position.y - win_half_y;
      max_y = current_pose_.pose.position.y + win_half_y;
    }

    const int grid_w = std::max(10, static_cast<int>(std::ceil((max_x - min_x) / resolution_)));
    const int grid_h = std::max(10, static_cast<int>(std::ceil((max_y - min_y) / resolution_)));
    grid.info.width = grid_w;
    grid.info.height = grid_h;

    const double raw_origin_x = min_x;
    const double raw_origin_y = min_y;
    // Keep exact map-aligned origin. Snapping to resolution can introduce a visible offset
    // between lanelet markers and raster cells near boundaries.
    grid.info.origin.position.x = raw_origin_x;
    grid.info.origin.position.y = raw_origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = sy;
    grid.info.origin.orientation.w = cy;

    // 2026-02-02 11:55: Default fill depends on mode (path wants high cost outside).
    int default_cell = -1;
    if (outside_value_ >= 0) {
      default_cell = outside_value_;
    } else if (cost_mode_ == "path") {
      // 2026-02-11: When lanelet mask mode is on, keep unknown outside lanes for lane-shaped rendering.
      // Otherwise keep legacy behavior (free before first path, then lethal outside path).
      default_cell = path_use_lanelet_mask_
        ? -1
        : (path_received_ ? lethal_value_ : free_value_);
    } else if (cost_mode_ == "bounds") {
      // HH_260305-00:00 Boundary mode should visualize only lane boundaries by default.
      // Keep non-boundary cells unknown so marker output is not a full filled lane polygon.
      default_cell = -1;
    }
    grid.data.assign(grid_w * grid_h, default_cell);
    // HH_260101 keep origin for rasterization
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;

    // HH_260101 fill free cells along lanelet centerlines only (narrow corridor)
    const double robot_yaw = has_pose_ ? yawFromPose(current_pose_) : 0.0;
    const double robot_cos = std::cos(robot_yaw);
    const double robot_sin = std::sin(robot_yaw);
    boundary_candidates_ = 0;
    boundary_rejected_clip_ = 0;
    boundary_written_ = 0;

    if (cost_mode_ == "path") {
      const std::vector<lanelet::BasicPolygon2d> * path_lanelet_polys_ptr = nullptr;
      bool using_all_lanelet_fallback = false;
      if (path_clip_to_lanelet_ || path_use_lanelet_mask_) {
        if (path_lanelet_only_ && path_received_ && path_.poses.size() > 1) {
          ensurePathLaneletCache();
          if (!cached_path_lanelet_polys_.empty()) {
            path_lanelet_polys_ptr = &cached_path_lanelet_polys_;
          } else {
            // HH_260318-00:00 Robustness fallback:
            // When route-lanelet matching fails temporarily after a goal update,
            // do not freeze old markers. Fall back to all lanelets so the new
            // global path strip can still be rasterized, while clip-to-lanelet
            // keeps cells inside legal lane polygons.
            path_lanelet_polys_ptr = &all_lanelet_polys_;
            using_all_lanelet_fallback = true;
          }
        } else {
          path_lanelet_polys_ptr = &all_lanelet_polys_;
        }
      }
      const std::vector<lanelet::BasicPolygon2d> * path_clip_polys_ptr =
        path_clip_to_lanelet_ ? path_lanelet_polys_ptr : nullptr;
      // HH_260313-00:00 Safety guard:
      // If path-lanelet-only clipping is requested but no lanelets are matched,
      // never paint the strip outside lane boundaries.
      const bool no_clip_lanelet_available =
        path_clip_to_lanelet_ &&
        path_lanelet_only_ &&
        path_clip_polys_ptr &&
        path_clip_polys_ptr->empty();
      if (using_all_lanelet_fallback) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "path lanelet match empty -> fallback to all lanelets (prevents stale global grid)");
      }
      if (no_clip_lanelet_available) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "path clip skipped: no matched lanelets for current path (lanelet_only=true)");
      }

      if (path_use_lanelet_mask_ && path_lanelet_polys_ptr && !path_lanelet_polys_ptr->empty()) {
        const int lane_cost = std::clamp(path_lane_base_value_, free_value_, lethal_value_);
        for (const auto & poly : *path_lanelet_polys_ptr) {
          if (path_use_pose_distance_gradient_ && has_pose_) {
            fillPolygonPoseGradient(poly, lane_cost, grid, false);
          } else {
            // Use center-point containment to avoid one-cell bleed outside lane boundaries.
            fillPolygonConst(poly, lane_cost, grid, false);
          }
        }
      }
      if (!no_clip_lanelet_available && path_received_ && path_.poses.size() > 1) {
        for (size_t i = 0; i + 1 < path_.poses.size(); ++i) {
          const auto & p0_msg = path_.poses[i].pose.position;
          const auto & p1_msg = path_.poses[i + 1].pose.position;
          const double dx = p1_msg.x - p0_msg.x;
          const double dy = p1_msg.y - p0_msg.y;
          const double len = std::hypot(dx, dy);
          if (len < 1e-3) {
            continue;
          }
          const double seg_dir_cos = dx / len;
          const double seg_dir_sin = dy / len;
          const double heading_dot = seg_dir_cos * robot_cos + seg_dir_sin * robot_sin;
          const double nx = -dy / len * centerline_half_width_;
          const double ny = dx / len * centerline_half_width_;
          lanelet::BasicPolygon2d strip{
            {p0_msg.x + nx, p0_msg.y + ny},
            {p0_msg.x - nx, p0_msg.y - ny},
            {p1_msg.x - nx, p1_msg.y - ny},
            {p1_msg.x + nx, p1_msg.y + ny}
          };
          fillPathStrip(
            strip, p0_msg, p1_msg, heading_dot < 0.0, robot_cos, robot_sin, grid,
            path_clip_polys_ptr, centerline_half_width_);
        }
      }
    } else if (cost_mode_ == "bounds") {
      for (const auto & ll : map_->laneletLayer) {
        lanelet::BasicPolygon2d lane_poly;
        if (!buildLaneletPolygon(ll, lane_poly)) {
          continue;
        }
        const auto & left = ll.leftBound();
        const auto & right = ll.rightBound();
        if (left.size() >= 2) {
          for (size_t i = 0; i + 1 < left.size(); ++i) {
            fillBoundaryStrip(
              left[i], left[i + 1], lethal_value_, grid,
              boundary_clip_to_lanelet_ ? &lane_poly : nullptr);
          }
        }
        if (right.size() >= 2) {
          for (size_t i = 0; i + 1 < right.size(); ++i) {
            fillBoundaryStrip(
              right[i], right[i + 1], lethal_value_, grid,
              boundary_clip_to_lanelet_ ? &lane_poly : nullptr);
          }
        }
      }
    } else if (cost_mode_ == "lanelet") {
      for (const auto & ll : map_->laneletLayer) {
        lanelet::BasicPolygon2d lane_poly;
        if (!buildLaneletPolygon(ll, lane_poly)) {
          continue;
        }
        fillPolygonConst(lane_poly, free_value_, grid, false);
        if (lanelet_boundary_value_ >= 0) {
          const int boundary_cost = std::clamp(lanelet_boundary_value_, 0, 100);
          const auto & left = ll.leftBound();
          const auto & right = ll.rightBound();
          if (left.size() >= 2) {
            for (size_t i = 0; i + 1 < left.size(); ++i) {
              fillBoundaryStrip(
                left[i], left[i + 1], boundary_cost, grid,
                boundary_clip_to_lanelet_ ? &lane_poly : nullptr);
            }
          }
          if (right.size() >= 2) {
            for (size_t i = 0; i + 1 < right.size(); ++i) {
              fillBoundaryStrip(
                right[i], right[i + 1], boundary_cost, grid,
                boundary_clip_to_lanelet_ ? &lane_poly : nullptr);
            }
          }
        }
      }
    } else {  // centerline (default)
      for (const auto & ll : map_->laneletLayer) {
        lanelet::BasicPolygon2d lane_poly;
        const bool has_lane_poly = buildLaneletPolygon(ll, lane_poly);
        if (has_lane_poly && centerline_lanelet_fill_value_ >= 0) {
          // HH_260316-00:00 Fill lanelet interior with soft base cost so global
          // connectivity is preserved even when centerline strips get sparse at bends.
          const int lane_base_cost = std::clamp(
            centerline_lanelet_fill_value_, free_value_, lethal_value_);
          fillPolygonConst(lane_poly, lane_base_cost, grid, false);
        }
        const auto & cl = ll.centerline();
        if (cl.size() < 2) {
          continue;
        }
        for (size_t i = 0; i + 1 < cl.size(); ++i) {
          const auto & p0 = cl[i];
          const auto & p1 = cl[i + 1];
          const double dx = p1.x() - p0.x();
          const double dy = p1.y() - p0.y();
          const double len = std::hypot(dx, dy);
          if (len < 1e-3) {
            continue;
          }
          const double lane_dir_cos = dx / len;
          const double lane_dir_sin = dy / len;
          const double heading_dot = lane_dir_cos * robot_cos + lane_dir_sin * robot_sin;
          const double nx = -dy / len * centerline_half_width_;
          const double ny = dx / len * centerline_half_width_;
          lanelet::BasicPolygon2d strip{
            {p0.x() + nx, p0.y() + ny},
            {p0.x() - nx, p0.y() - ny},
            {p1.x() - nx, p1.y() - ny},
            {p1.x() + nx, p1.y() + ny}
          };
          fillCenterlineStrip(
            strip, p0, p1, heading_dot < 0.0, robot_cos, robot_sin, grid,
            centerline_use_distance_gradient_,
            (centerline_clip_to_lanelet_ && has_lane_poly) ? &lane_poly : nullptr);
        }
      }
    }

    grid.header.stamp = now();
    last_grid_ = grid;
    grid_pub_->publish(grid);
    publishAvgMapGrid(grid, "lanelet cost grid published");
    if (debug_boundary_stats_ && (cost_mode_ == "bounds" || lanelet_boundary_value_ >= 0)) {
      const double rej_ratio = boundary_candidates_ > 0
        ? static_cast<double>(boundary_rejected_clip_) / static_cast<double>(boundary_candidates_)
        : 0.0;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "boundary stats: candidates=%ld written=%ld clipped_outside=%ld reject_ratio=%.3f",
        boundary_candidates_, boundary_written_, boundary_rejected_clip_, rej_ratio);
    }
    if (debug_coverage_stats_) {
      logCoverageStats(grid);
    }
    RCLCPP_DEBUG(get_logger(),
      "published lanelet cost grid (%ux%u, res=%.2f) frame=%s",
      grid.info.width, grid.info.height, grid.info.resolution, grid.header.frame_id.c_str());
  }

  // Implements `requestBuild` behavior.
  void requestBuild(bool force)
  {
    if (!map_ || (!has_pose_ && requiresPoseForBuild())) {
      return;
    }
    if (
      cost_mode_ == "path" &&
      (!path_received_ || path_.poses.size() < 2) &&
      !allow_build_without_path_)
    {
      ++skip_no_path_count_;
      return;
    }
    if (!force && min_rebuild_period_sec_ > 0.0 && last_build_time_.nanoseconds() > 0) {
      const double dt = (now() - last_build_time_).seconds();
      if (dt < min_rebuild_period_sec_) {
        ++skip_throttle_count_;
        return;
      }
    }
    const auto build_t0 = std::chrono::steady_clock::now();
    buildGrid();
    const auto build_t1 = std::chrono::steady_clock::now();
    if (debug_build_timing_) {
      const double ms = std::chrono::duration<double, std::milli>(build_t1 - build_t0).count();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "build timing: mode=%s size=%ux%u dt=%.2fms",
        cost_mode_.c_str(),
        last_grid_.info.width, last_grid_.info.height,
        ms);
    }
    ++build_ok_count_;
    has_built_grid_ = true;
    last_build_time_ = now();
  }

  // Updates `RepublishTimer` state.
  void updateRepublishTimer()
  {
    republish_timer_.reset();
    if (republish_period_ <= 0.0) {
      return;
    }
    republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(republish_period_)),
      std::bind(&LaneletCostGridNode::onRepublishTimer, this));
  }

  // Implements `clearPublishedGrid` behavior.
  void clearPublishedGrid()
  {
    avg_msgs::msg::OccupancyGrid empty;
    empty.header.stamp = now();
    empty.header.frame_id = frame_id_;
    empty.info.resolution = resolution_;
    empty.info.width = 0;
    empty.info.height = 0;
    empty.info.origin.position.x = 0.0;
    empty.info.origin.position.y = 0.0;
    empty.info.origin.position.z = 0.0;
    empty.info.origin.orientation.w = 1.0;
    last_grid_ = empty;
    has_built_grid_ = true;
    grid_pub_->publish(last_grid_);
    publishAvgMapGrid(last_grid_, "lanelet cost grid cleared");
  }

  // Publishes `AvgMapGrid` output.
  void publishAvgMapGrid(const avg_msgs::msg::OccupancyGrid & grid, const std::string & message)
  {
    if (!publish_map_diagnostic_ || !avg_map_pub_) {
      return;
    }
    avg_msgs::msg::AvgMapMsgs msg;
    msg.stamp = grid.header.stamp;
    msg.health.stamp = grid.header.stamp;
    msg.health.module_name = "map";
    msg.health.level = avg_msgs::msg::ModuleHealth::OK;
    msg.health.message = message;
    msg.lanelet_cost_grid = grid;
    avg_map_pub_->publish(msg);
  }

  // Implements `requiresPoseForBuild` behavior.
  bool requiresPoseForBuild() const
  {
    const bool window_requires_pose = !lock_window_ && !use_path_bbox_;
    bool pose_dependent_cost = false;
    if (cost_mode_ == "path") {
      pose_dependent_cost =
        direction_penalty_ != 0 ||
        backward_penalty_ != 0 ||
        path_use_pose_distance_gradient_;
    } else if (cost_mode_ == "centerline") {
      pose_dependent_cost =
        direction_penalty_ != 0 ||
        backward_penalty_ != 0 ||
        centerline_use_distance_gradient_;
    }
    return window_requires_pose || pose_dependent_cost;
  }

  // Computes `RasterPadding` values.
  double computeRasterPadding() const
  {
    if (cost_mode_ == "bounds") {
      return std::max(boundary_half_width_, resolution_);
    }
    if (cost_mode_ == "path" || cost_mode_ == "centerline") {
      return std::max(centerline_half_width_, resolution_);
    }
    return std::max(0.5 * resolution_, 0.01);
  }

  // Implements `fillPolygon` behavior.
  void fillPolygon(
    const lanelet::BasicPolygon2d & poly, bool opposite_heading,
    double robot_cos, double robot_sin,
    avg_msgs::msg::OccupancyGrid & grid,
    bool use_distance_gradient)
  {
    // simple rasterization by bounding box scan; no anti-aliasing
    if (poly.empty()) return;
    double min_x = poly[0].x(), max_x = poly[0].x();
    double min_y = poly[0].y(), max_y = poly[0].y();
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    const int ix_min = std::max(0, static_cast<int>((min_x - origin_x_) / resolution_));
    const int ix_max = std::min(grid_w - 1, static_cast<int>((max_x - origin_x_) / resolution_));
    const int iy_min = std::max(0, static_cast<int>((min_y - origin_y_) / resolution_));
    const int iy_max = std::min(grid_h - 1, static_cast<int>((max_y - origin_y_) / resolution_));

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        double wx = origin_x_ + (ix + 0.5) * resolution_;
        double wy = origin_y_ + (iy + 0.5) * resolution_;
        if (cellIntersectsPolygon(poly, wx, wy)) {
          int cost = free_value_;
          if (use_distance_gradient) {
            const double dist =
              std::hypot(wx - current_pose_.pose.position.x, wy - current_pose_.pose.position.y);
            const double norm = gradient_range_ > 1e-3 ? dist / gradient_range_ : 0.0;
            cost = std::clamp(
              static_cast<int>(norm * lethal_value_), free_value_, lethal_value_);
          }
          if (has_pose_ && opposite_heading) {
            cost = std::clamp(cost + direction_penalty_, free_value_, lethal_value_);
          }
          // HH_260102 penalize cells behind robot heading to discourage reverse driving
          if (has_pose_) {
            const double vx = wx - current_pose_.pose.position.x;
            const double vy = wy - current_pose_.pose.position.y;
            const double forward_dot = vx * robot_cos + vy * robot_sin;
            if (forward_dot < 0.0) {
              cost = std::clamp(cost + backward_penalty_, free_value_, lethal_value_);
            }
          }
          grid.data[iy * grid_w + ix] = cost;
        }
      }
    }
  }

  // Implements `fillBoundaryStrip` behavior.
  void fillBoundaryStrip(
    const lanelet::ConstPoint3d & p0,
    const lanelet::ConstPoint3d & p1,
    int value,
    avg_msgs::msg::OccupancyGrid & grid,
    const lanelet::BasicPolygon2d * clip_poly = nullptr)
  {
    const double dx = p1.x() - p0.x();
    const double dy = p1.y() - p0.y();
    const double len = std::hypot(dx, dy);
    if (len < 1e-3) {
      return;
    }
    const double nx = -dy / len * boundary_half_width_;
    const double ny = dx / len * boundary_half_width_;
    lanelet::BasicPolygon2d strip{
      {p0.x() + nx, p0.y() + ny},
      {p0.x() - nx, p0.y() - ny},
      {p1.x() - nx, p1.y() - ny},
      {p1.x() + nx, p1.y() + ny}
    };
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    double min_x = strip[0].x(), max_x = strip[0].x();
    double min_y = strip[0].y(), max_y = strip[0].y();
    for (const auto & p : strip) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    const int ix_min = std::max(0, static_cast<int>((min_x - origin_x_) / resolution_));
    const int ix_max = std::min(grid_w - 1, static_cast<int>((max_x - origin_x_) / resolution_));
    const int iy_min = std::max(0, static_cast<int>((min_y - origin_y_) / resolution_));
    const int iy_max = std::min(grid_h - 1, static_cast<int>((max_y - origin_y_) / resolution_));

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double wx = origin_x_ + (ix + 0.5) * resolution_;
        const double wy = origin_y_ + (iy + 0.5) * resolution_;
        if (!cellMostlyInsidePolygon(strip, wx, wy, cell_inside_min_hits_boundary_)) {
          continue;
        }
        ++boundary_candidates_;
        // HH_260305-00:00 Use center-point containment for clip checks.
        // This avoids one-cell outward bleed on tight corners/boundaries.
        if (clip_poly && !lanelet::geometry::within(lanelet::BasicPoint2d(wx, wy), *clip_poly)) {
          ++boundary_rejected_clip_;
          continue;
        }
        grid.data[iy * grid_w + ix] = value;
        ++boundary_written_;
      }
    }
  }

  // Implements `fillLaneletArea` behavior.
  void fillLaneletArea(
    const lanelet::ConstLanelet & ll, int value,
    avg_msgs::msg::OccupancyGrid & grid)
  {
    lanelet::BasicPolygon2d poly;
    if (!buildLaneletPolygon(ll, poly)) {
      return;
    }
    // Use center-point containment for full-lanelet fill to avoid one-cell bleed outside boundaries.
    fillPolygonConst(poly, value, grid, false);
  }

  // Implements `buildLaneletPolygon` behavior.
  bool buildLaneletPolygon(const lanelet::ConstLanelet & ll, lanelet::BasicPolygon2d & poly) const
  {
    // HH_260317-00:00 Use lanelet-native polygon2d() instead of manual
    // left/right stitching. This avoids winding/order inconsistencies and
    // stabilizes lanelet-inside checks for clipping/coverage validation.
    const auto lane_poly = ll.polygon2d();
    if (lane_poly.size() < 3) {
      return false;
    }

    poly.clear();
    poly.reserve(lane_poly.size());
    for (const auto & p : lane_poly) {
      poly.emplace_back(p.x(), p.y());
    }

    // Remove duplicated closure vertex when present.
    if (poly.size() >= 2) {
      const auto & first = poly.front();
      const auto & last = poly.back();
      if (std::hypot(first.x() - last.x(), first.y() - last.y()) < 1e-9) {
        poly.pop_back();
      }
    }
    if (poly.size() < 3) {
      poly.clear();
      return false;
    }
    return true;
  }

  // Implements `buildAllLaneletPolygons` behavior.
  void buildAllLaneletPolygons()
  {
    all_lanelets_.clear();
    all_lanelet_ids_.clear();
    all_lanelet_bounds_.clear();
    all_lanelet_polys_.clear();
    if (!map_) {
      return;
    }
    all_lanelets_.reserve(map_->laneletLayer.size());
    all_lanelet_ids_.reserve(map_->laneletLayer.size());
    all_lanelet_bounds_.reserve(map_->laneletLayer.size());
    all_lanelet_polys_.reserve(map_->laneletLayer.size());
    for (const auto & ll : map_->laneletLayer) {
      lanelet::BasicPolygon2d poly;
      if (buildLaneletPolygon(ll, poly)) {
        double min_x = poly.front().x();
        double max_x = poly.front().x();
        double min_y = poly.front().y();
        double max_y = poly.front().y();
        for (const auto & p : poly) {
          min_x = std::min(min_x, p.x());
          max_x = std::max(max_x, p.x());
          min_y = std::min(min_y, p.y());
          max_y = std::max(max_y, p.y());
        }
        all_lanelets_.push_back(ll);
        all_lanelet_ids_.push_back(ll.id());
        all_lanelet_bounds_.push_back({min_x, max_x, min_y, max_y});
        all_lanelet_polys_.push_back(std::move(poly));
      }
    }
    invalidatePathLaneletCache();
  }

  // Implements `collectPathLanelets` behavior.
  std::vector<lanelet::ConstLanelet> collectPathLanelets() const
  {
    std::vector<lanelet::ConstLanelet> result;
    if (!map_ || path_.poses.empty() || all_lanelet_polys_.empty()) {
      return result;
    }

    std::unordered_set<lanelet::Id> ids;
    const double sample_step = std::max(0.02, path_lanelet_sample_step_);
    for (size_t i = 0; i + 1 < path_.poses.size(); ++i) {
      const auto & p0 = path_.poses[i].pose.position;
      const auto & p1 = path_.poses[i + 1].pose.position;
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      const double len = std::hypot(dx, dy);
      const int samples = std::max(1, static_cast<int>(std::ceil(len / sample_step)));
      for (int s = 0; s <= samples; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(samples);
        const lanelet::BasicPoint2d p(p0.x + t * dx, p0.y + t * dy);
        bool contained = false;
        for (std::size_t li = 0; li < all_lanelet_polys_.size(); ++li) {
          const auto & bounds = all_lanelet_bounds_[li];
          if (
            p.x() < bounds[0] || p.x() > bounds[1] ||
            p.y() < bounds[2] || p.y() > bounds[3])
          {
            continue;
          }
          if (lanelet::geometry::within(p, all_lanelet_polys_[li])) {
            ids.insert(all_lanelet_ids_[li]);
            contained = true;
          }
        }
        if (contained) {
          continue;
        }
        // HH_260305-00:00 Keep fallback matching conservative to avoid
        // unrelated lanelets appearing as sparse remote cost patches.
        if (path_lanelet_allow_nearest_fallback_) {
          const auto nearest = lanelet::geometry::findNearest(map_->laneletLayer, p, 1);
          if (!nearest.empty() && nearest.front().first <= path_lanelet_match_max_dist_) {
            ids.insert(nearest.front().second.id());
          }
        }
      }
    }

    result.reserve(ids.size());
    for (std::size_t li = 0; li < all_lanelets_.size(); ++li) {
      if (ids.find(all_lanelet_ids_[li]) != ids.end()) {
        result.emplace_back(all_lanelets_[li]);
      }
    }
    return result;
  }

  // Implements `ensurePathLaneletCache` behavior.
  void ensurePathLaneletCache()
  {
    if (cached_path_lanelets_valid_) {
      return;
    }
    cached_path_lanelets_.clear();
    cached_path_lanelet_polys_.clear();
    if (path_received_ && path_.poses.size() > 1) {
      cached_path_lanelets_ = collectPathLanelets();
      cached_path_lanelet_polys_ = toLaneletPolygons(cached_path_lanelets_);
    }
    cached_path_lanelets_valid_ = true;
  }

  // Implements `invalidatePathLaneletCache` behavior.
  void invalidatePathLaneletCache()
  {
    cached_path_lanelets_valid_ = false;
    cached_path_lanelets_.clear();
    cached_path_lanelet_polys_.clear();
  }

  // Implements `fillPathStrip` behavior.
  void fillPathStrip(
    const lanelet::BasicPolygon2d & poly,
    const avg_msgs::msg::Point & p0,
    const avg_msgs::msg::Point & p1,
    bool opposite_heading,
    double robot_cos, double robot_sin,
    avg_msgs::msg::OccupancyGrid & grid,
    const std::vector<lanelet::BasicPolygon2d> * clip_polys,
    double strip_half_width)
  {
    if (poly.empty()) return;
    double min_x = poly[0].x(), max_x = poly[0].x();
    double min_y = poly[0].y(), max_y = poly[0].y();
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    const int ix_min = std::max(0, static_cast<int>((min_x - origin_x_) / resolution_));
    const int ix_max = std::min(grid_w - 1, static_cast<int>((max_x - origin_x_) / resolution_));
    const int iy_min = std::max(0, static_cast<int>((min_y - origin_y_) / resolution_));
    const int iy_max = std::min(grid_h - 1, static_cast<int>((max_y - origin_y_) / resolution_));

    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double vlen2 = vx * vx + vy * vy;
    const double half_width = std::max(strip_half_width, resolution_);
    const int path_lane_cap = std::clamp(path_lane_base_value_, free_value_, lethal_value_);

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double wx = origin_x_ + (ix + 0.5) * resolution_;
        const double wy = origin_y_ + (iy + 0.5) * resolution_;
        if (!cellMostlyInsidePolygon(poly, wx, wy, cell_inside_min_hits_path_)) {
          continue;
        }
        if (path_clip_to_lanelet_ && clip_polys && !clip_polys->empty()) {
          // HH_260313-00:00 Strict multi-sample clip prevents strip bleed outside boundaries.
          if (!cellMostlyInsideAnyPolygon(wx, wy, *clip_polys, cell_inside_min_hits_boundary_)) {
            continue;
          }
        }
        // HH_260313-00:00 Blend pose-distance and lateral-distance costs.
        // This keeps centerline cheapest while preserving realtime pose updates.
        double weighted_norm_sum = 0.0;
        double weight_sum = 0.0;
        if (path_use_pose_distance_gradient_ && has_pose_ && path_pose_cost_weight_ > 1e-9) {
          const double pose_dist =
            std::hypot(wx - current_pose_.pose.position.x, wy - current_pose_.pose.position.y);
          const double pose_norm = gradient_range_ > 1e-3
            ? std::clamp(pose_dist / gradient_range_, 0.0, 1.0)
            : 0.0;
          weighted_norm_sum += path_pose_cost_weight_ * pose_norm;
          weight_sum += path_pose_cost_weight_;
        }
        if (path_use_lateral_gradient_ && path_lateral_cost_weight_ > 1e-9 && vlen2 > 1e-6) {
          const double t = std::clamp(((wx - p0.x) * vx + (wy - p0.y) * vy) / vlen2, 0.0, 1.0);
          const double px = p0.x + t * vx;
          const double py = p0.y + t * vy;
          const double lateral_dist = std::hypot(wx - px, wy - py);
          const double lateral_norm = std::clamp(lateral_dist / half_width, 0.0, 1.0);
          weighted_norm_sum += path_lateral_cost_weight_ * lateral_norm;
          weight_sum += path_lateral_cost_weight_;
        }
        const double blended_norm = weight_sum > 1e-9 ? (weighted_norm_sum / weight_sum) : 0.0;
        int cost = static_cast<int>(
          free_value_ + blended_norm * (path_lane_cap - free_value_));
        if (has_pose_ && opposite_heading) {
          cost = std::clamp(cost + direction_penalty_, free_value_, lethal_value_);
        }
        if (has_pose_) {
          const double fvx = wx - current_pose_.pose.position.x;
          const double fvy = wy - current_pose_.pose.position.y;
          const double forward_dot = fvx * robot_cos + fvy * robot_sin;
          if (forward_dot < 0.0) {
            cost = std::clamp(cost + backward_penalty_, free_value_, lethal_value_);
          }
        }
        auto & cell = grid.data[iy * grid_w + ix];
        if (cell < 0 || cost < cell) {
          cell = cost;
        }
      }
    }
  }

  // Implements `fillPolygonPoseGradient` behavior.
  void fillPolygonPoseGradient(
    const lanelet::BasicPolygon2d & poly, int max_value,
    avg_msgs::msg::OccupancyGrid & grid,
    bool use_cell_intersection = true)
  {
    if (poly.empty() || !has_pose_) {
      return;
    }
    double min_x = poly[0].x(), max_x = poly[0].x();
    double min_y = poly[0].y(), max_y = poly[0].y();
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    const int ix_min = std::max(0, static_cast<int>((min_x - origin_x_) / resolution_));
    const int ix_max = std::min(grid_w - 1, static_cast<int>((max_x - origin_x_) / resolution_));
    const int iy_min = std::max(0, static_cast<int>((min_y - origin_y_) / resolution_));
    const int iy_max = std::min(grid_h - 1, static_cast<int>((max_y - origin_y_) / resolution_));

    const int clamped_max = std::clamp(max_value, free_value_, lethal_value_);
    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double wx = origin_x_ + (ix + 0.5) * resolution_;
        const double wy = origin_y_ + (iy + 0.5) * resolution_;
        const bool hit = use_cell_intersection
          ? cellIntersectsPolygon(poly, wx, wy)
          : cellMostlyInsidePolygon(poly, wx, wy, cell_inside_min_hits_path_);
        if (!hit) {
          continue;
        }
        const double pose_dist =
          std::hypot(wx - current_pose_.pose.position.x, wy - current_pose_.pose.position.y);
        const double pose_norm = gradient_range_ > 1e-3
          ? std::clamp(pose_dist / gradient_range_, 0.0, 1.0)
          : 0.0;
        const int cost = static_cast<int>(
          free_value_ + pose_norm * (clamped_max - free_value_));
        grid.data[iy * grid_w + ix] = cost;
      }
    }
  }

  // Implements `fillCenterlineStrip` behavior.
  void fillCenterlineStrip(
    const lanelet::BasicPolygon2d & poly,
    const lanelet::ConstPoint3d & p0,
    const lanelet::ConstPoint3d & p1,
    bool opposite_heading,
    double robot_cos, double robot_sin,
    avg_msgs::msg::OccupancyGrid & grid,
    bool use_lateral_gradient,
    const lanelet::BasicPolygon2d * clip_poly = nullptr)
  {
    if (poly.empty()) {
      return;
    }
    double min_x = poly[0].x(), max_x = poly[0].x();
    double min_y = poly[0].y(), max_y = poly[0].y();
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    const int ix_min = std::max(0, static_cast<int>((min_x - origin_x_) / resolution_));
    const int ix_max = std::min(grid_w - 1, static_cast<int>((max_x - origin_x_) / resolution_));
    const int iy_min = std::max(0, static_cast<int>((min_y - origin_y_) / resolution_));
    const int iy_max = std::min(grid_h - 1, static_cast<int>((max_y - origin_y_) / resolution_));

    const double sx = p1.x() - p0.x();
    const double sy = p1.y() - p0.y();
    const double vlen2 = sx * sx + sy * sy;
    const double half_width = std::max(1e-3, centerline_half_width_);

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double wx = origin_x_ + (ix + 0.5) * resolution_;
        const double wy = origin_y_ + (iy + 0.5) * resolution_;
        if (!cellMostlyInsidePolygon(poly, wx, wy, cell_inside_min_hits_path_)) {
          continue;
        }
        // HH_260316-00:00 Keep centerline strip strictly lanelet-internal.
        if (clip_poly && !lanelet::geometry::within(lanelet::BasicPoint2d(wx, wy), *clip_poly)) {
          continue;
        }
        int cost = free_value_;
        if (use_lateral_gradient && vlen2 > 1e-6) {
          const double t = std::clamp(((wx - p0.x()) * sx + (wy - p0.y()) * sy) / vlen2, 0.0, 1.0);
          const double px = p0.x() + t * sx;
          const double py = p0.y() + t * sy;
          const double dist = std::hypot(wx - px, wy - py);
          const double norm = std::clamp(dist / half_width, 0.0, 1.0);
          cost = static_cast<int>(free_value_ + norm * (lethal_value_ - free_value_));
        }
        if (has_pose_ && opposite_heading) {
          cost = std::clamp(cost + direction_penalty_, free_value_, lethal_value_);
        }
        if (has_pose_) {
          const double fvx = wx - current_pose_.pose.position.x;
          const double fvy = wy - current_pose_.pose.position.y;
          const double forward_dot = fvx * robot_cos + fvy * robot_sin;
          if (forward_dot < 0.0) {
            cost = std::clamp(cost + backward_penalty_, free_value_, lethal_value_);
          }
        }
        auto & cell = grid.data[iy * grid_w + ix];
        if (cell < 0 || cost < cell) {
          cell = cost;
        }
      }
    }
  }

  // Implements `toLaneletPolygons` behavior.
  std::vector<lanelet::BasicPolygon2d> toLaneletPolygons(
    const std::vector<lanelet::ConstLanelet> & lanelets) const
  {
    std::vector<lanelet::BasicPolygon2d> polys;
    polys.reserve(lanelets.size());
    for (const auto & ll : lanelets) {
      lanelet::BasicPolygon2d poly;
      if (!buildLaneletPolygon(ll, poly)) {
        continue;
      }
      polys.emplace_back(std::move(poly));
    }
    return polys;
  }

  // Implements `fillPolygonConst` behavior.
  void fillPolygonConst(
    const lanelet::BasicPolygon2d & poly, int value,
    avg_msgs::msg::OccupancyGrid & grid,
    bool use_cell_intersection = true)
  {
    if (poly.empty()) return;
    double min_x = poly[0].x(), max_x = poly[0].x();
    double min_y = poly[0].y(), max_y = poly[0].y();
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    const int ix_min = std::max(0, static_cast<int>((min_x - origin_x_) / resolution_));
    const int ix_max = std::min(grid_w - 1, static_cast<int>((max_x - origin_x_) / resolution_));
    const int iy_min = std::max(0, static_cast<int>((min_y - origin_y_) / resolution_));
    const int iy_max = std::min(grid_h - 1, static_cast<int>((max_y - origin_y_) / resolution_));

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double wx = origin_x_ + (ix + 0.5) * resolution_;
        const double wy = origin_y_ + (iy + 0.5) * resolution_;
        const bool hit = use_cell_intersection
          ? cellIntersectsPolygon(poly, wx, wy)
          : lanelet::geometry::within(lanelet::BasicPoint2d(wx, wy), poly);
        if (hit) {
          grid.data[iy * grid_w + ix] = value;
        }
      }
    }
  }

  // Implements `withinAnyPolygon` behavior.
  bool withinAnyPolygon(
    const lanelet::BasicPoint2d & p,
    const std::vector<lanelet::BasicPolygon2d> & polys) const
  {
    for (const auto & poly : polys) {
      if (lanelet::geometry::within(p, poly)) {
        return true;
      }
    }
    return false;
  }

  // Implements `cellMostlyInsideAnyPolygon` behavior.
  bool cellMostlyInsideAnyPolygon(
    double cx, double cy,
    const std::vector<lanelet::BasicPolygon2d> & polys,
    int min_hits = -1) const
  {
    for (const auto & poly : polys) {
      if (cellMostlyInsidePolygon(poly, cx, cy, min_hits)) {
        return true;
      }
    }
    return false;
  }

  // Implements `logCoverageStats` behavior.
  void logCoverageStats(const avg_msgs::msg::OccupancyGrid & grid)
  {
    if (all_lanelet_polys_.empty()) {
      return;
    }
    const int stride = std::max(1, debug_coverage_stride_);
    const int grid_w = static_cast<int>(grid.info.width);
    const int grid_h = static_cast<int>(grid.info.height);
    const double res = grid.info.resolution;
    const double ox = grid.info.origin.position.x;
    const double oy = grid.info.origin.position.y;

    int64_t sampled_known = 0;
    int64_t sampled_outside = 0;
    for (int iy = 0; iy < grid_h; iy += stride) {
      for (int ix = 0; ix < grid_w; ix += stride) {
        const int8_t v = grid.data[static_cast<size_t>(iy) * static_cast<size_t>(grid_w) +
          static_cast<size_t>(ix)];
        if (v < 0 || v < debug_coverage_min_value_) {
          continue;
        }
        ++sampled_known;
        const double wx = ox + (static_cast<double>(ix) + 0.5) * res;
        const double wy = oy + (static_cast<double>(iy) + 0.5) * res;
        if (!withinAnyPolygon(lanelet::BasicPoint2d(wx, wy), all_lanelet_polys_)) {
          ++sampled_outside;
        }
      }
    }

    if (sampled_known <= 0) {
      return;
    }
    const double ratio = static_cast<double>(sampled_outside) / static_cast<double>(sampled_known);
    if (sampled_outside > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "coverage stats: sampled_known=%ld sampled_outside=%ld outside_ratio=%.4f stride=%d min_value=%d",
        sampled_known, sampled_outside, ratio, stride, debug_coverage_min_value_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 4000,
        "coverage stats: sampled_known=%ld sampled_outside=0 outside_ratio=0.0000 stride=%d min_value=%d",
        sampled_known, stride, debug_coverage_min_value_);
    }
  }

  // Implements `cellIntersectsPolygon` behavior.
  bool cellIntersectsPolygon(
    const lanelet::BasicPolygon2d & poly, double cx, double cy) const
  {
    const double h = 0.5 * resolution_;
    const std::array<lanelet::BasicPoint2d, 5> samples{
      lanelet::BasicPoint2d(cx, cy),
      lanelet::BasicPoint2d(cx - h, cy - h),
      lanelet::BasicPoint2d(cx - h, cy + h),
      lanelet::BasicPoint2d(cx + h, cy - h),
      lanelet::BasicPoint2d(cx + h, cy + h)
    };
    for (const auto & s : samples) {
      if (lanelet::geometry::within(s, poly)) {
        return true;
      }
    }
    return false;
  }

  // Implements `cellMostlyInsidePolygon` behavior.
  bool cellMostlyInsidePolygon(
    const lanelet::BasicPolygon2d & poly, double cx, double cy, int min_hits = -1) const
  {
    if (min_hits < 0) {
      min_hits = cell_inside_min_hits_;
    }
    min_hits = std::clamp(min_hits, 1, 9);
    const double h = 0.5 * resolution_;
    const std::array<lanelet::BasicPoint2d, 9> samples{
      lanelet::BasicPoint2d(cx, cy),
      lanelet::BasicPoint2d(cx - h, cy - h),
      lanelet::BasicPoint2d(cx - h, cy),
      lanelet::BasicPoint2d(cx - h, cy + h),
      lanelet::BasicPoint2d(cx, cy - h),
      lanelet::BasicPoint2d(cx, cy + h),
      lanelet::BasicPoint2d(cx + h, cy - h),
      lanelet::BasicPoint2d(cx + h, cy),
      lanelet::BasicPoint2d(cx + h, cy + h)
    };
    int inside = 0;
    for (const auto & s : samples) {
      if (lanelet::geometry::within(s, poly)) {
        ++inside;
      }
    }
    return inside >= min_hits;
  }

  // Implements `yawFromPose` behavior.
  double yawFromPose(const avg_msgs::msg::PoseStamped & pose) const
  {
    const auto & q = pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // HH_260125 Transform helper: ensure all poses are in map frame.
  bool transformToMap(
    const avg_msgs::msg::PoseStamped & in,
    avg_msgs::msg::PoseStamped & out)
  {
    if (in.header.frame_id.empty() || in.header.frame_id == frame_id_) {
      out = in;
      out.header.frame_id = frame_id_;
      return true;
    }
    if (!tf_buffer_) {
      // 2026-01-27 17:45: Remove HH tags from runtime logs.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tf buffer not ready, cannot transform from %s to %s",
        in.header.frame_id.c_str(), frame_id_.c_str());
      return false;
    }
    try {
      out = tf_buffer_->transform(in, frame_id_, tf2::durationFromSec(0.1));
      out.header.frame_id = frame_id_;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to transform pose from %s to %s: %s",
        in.header.frame_id.c_str(), frame_id_.c_str(), ex.what());
      return false;
    }
  }

  std::string map_path_;
  std::string frame_id_;
  double resolution_;
  int window_width_;
  int window_height_;
  double origin_x_;
  double origin_y_;
  double centerline_half_width_;
  double boundary_half_width_;
  int lanelet_boundary_value_;
  bool boundary_clip_to_lanelet_;
  bool debug_boundary_stats_;
  bool debug_coverage_stats_{false};
  int debug_coverage_stride_{3};
  int debug_coverage_min_value_{0};
  int64_t boundary_candidates_{0};
  int64_t boundary_rejected_clip_{0};
  int64_t boundary_written_{0};
  double origin_lat_;
  double origin_lon_;
  double origin_alt_;
  std::string projector_type_;
  int free_value_;
  int lethal_value_;
  int outside_value_;
  double gradient_range_;
  int direction_penalty_;
  int backward_penalty_;
  bool centerline_use_distance_gradient_;
  bool centerline_clip_to_lanelet_{true};
  int centerline_lanelet_fill_value_{-1};
  bool path_use_lanelet_mask_;
  int path_lane_base_value_;
  bool path_lanelet_only_;
  double path_lanelet_match_max_dist_;
  bool path_lanelet_allow_nearest_fallback_;
  double path_lanelet_sample_step_;
  bool path_clip_to_lanelet_;
  bool path_use_pose_distance_gradient_;
  bool path_use_lateral_gradient_;
  double path_pose_cost_weight_;
  double path_lateral_cost_weight_;
  bool ignore_empty_path_{true};
  bool clear_path_on_goal_{false};
  bool rebuild_on_pose_{true};
  bool rebuild_on_path_{true};
  bool rebuild_on_timer_{false};
  double min_rebuild_period_sec_{0.0};
  bool allow_build_without_path_{false};
  bool drop_stale_path_after_goal_{true};
  double path_goal_stamp_slack_sec_{0.10};
  double fresh_path_goal_match_tolerance_m_{2.0};
  bool debug_rebuild_stats_{false};
  bool debug_build_timing_{false};
  int cell_inside_min_hits_{5};
  int cell_inside_min_hits_path_{5};
  int cell_inside_min_hits_boundary_{5};
  int64_t pose_cb_count_{0};
  int64_t path_cb_count_{0};
  int64_t build_ok_count_{0};
  int64_t skip_no_path_count_{0};
  int64_t skip_throttle_count_{0};
  bool path_qos_transient_local_{false};
  std::vector<lanelet::ConstLanelet> all_lanelets_;
  std::vector<lanelet::Id> all_lanelet_ids_;
  std::vector<std::array<double, 4>> all_lanelet_bounds_;
  std::vector<lanelet::BasicPolygon2d> all_lanelet_polys_;
  std::vector<lanelet::ConstLanelet> cached_path_lanelets_;
  std::vector<lanelet::BasicPolygon2d> cached_path_lanelet_polys_;
  bool cached_path_lanelets_valid_{false};
  double grid_yaw_;
  std::string cost_mode_;
  bool use_path_bbox_;
  bool lock_window_;
  bool use_map_bbox_;
  bool ignore_invalid_map_;
  std::string pose_topic_;
  std::string path_topic_;
  std::string path_fallback_topic_;
  std::string goal_topic_;
  double primary_path_timeout_sec_{1.0};
  double goal_fallback_holdoff_sec_{0.6};
  double stale_path_timeout_sec_{0.0};
  double republish_period_;
  std::string output_topic_;
  bool publish_map_diagnostic_{false};
  std::string map_diagnostic_topic_{"/map/diagnostic"};

  bool map_bounds_valid_{false};
  double map_min_x_{0.0};
  double map_min_y_{0.0};
  double map_max_x_{0.0};
  double map_max_y_{0.0};

  lanelet::LaneletMapPtr map_;
  avg_msgs::msg::PoseStamped current_pose_;
  avg_msgs::msg::PoseStamped latest_goal_pose_;
  avg_msgs::msg::Path path_;
  rclcpp::Time last_primary_path_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_any_path_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time fallback_holdoff_until_{0, 0, RCL_ROS_TIME};
  bool waiting_fresh_path_after_goal_{false};
  bool has_latest_goal_pose_{false};
  bool has_pose_{false};
  bool path_received_{false};
  bool has_built_grid_{false};
  avg_msgs::msg::OccupancyGrid last_grid_;
  rclcpp::Time last_build_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<avg_msgs::msg::Path>::SharedPtr path_fallback_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<avg_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgMapMsgs>::SharedPtr avg_map_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneletCostGridNode>());
  rclcpp::shutdown();
  return 0;
}
