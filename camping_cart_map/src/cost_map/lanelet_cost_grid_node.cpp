#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include "camping_cart_map/custom_regulatory_elements.hpp"  // HH_260101 register speed_bump
#include <cmath>
#include <algorithm>

#include <string>
#include <vector>
#include <memory>
#include <filesystem>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    origin_lat_ = declare_parameter<double>("offset_lat", 0.0);   // HH_260101 map origin for projection
    origin_lon_ = declare_parameter<double>("offset_lon", 0.0);
    origin_alt_ = declare_parameter<double>("offset_alt", 0.0);
    free_value_ = declare_parameter<int>("free_value", 0);
    lethal_value_ = declare_parameter<int>("lethal_value", 100);
    // 2026-02-02 11:55: Cost mode controls how the grid is rasterized.
    //   centerline = low-cost corridor around lanelet centerlines (default)
    //   bounds     = high-cost thin strips along left/right boundaries only
    //   path       = low-cost corridor along path, everything else high
    cost_mode_ = declare_parameter<std::string>("cost_mode", "centerline");
    boundary_half_width_ = declare_parameter<double>("boundary_half_width", 0.2);
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
    grid_yaw_ = declare_parameter<double>("grid_yaw", 0.0);  // HH_260103 manual yaw (rad) for OccupancyGrid orientation
    use_path_bbox_ = declare_parameter<bool>("use_path_bbox", false);  // HH_260103 false -> robot-centered window, true -> path bbox window
    lock_window_ = declare_parameter<bool>("lock_window", false);  // HH_260126 keep grid window fixed (use origin_x/y + width/height)
    // 2026-01-30: Allow fixed window to follow map bounds without hardcoding width/height.
    use_map_bbox_ = declare_parameter<bool>("use_map_bbox", false);
    ignore_invalid_map_ = declare_parameter<bool>("ignore_invalid_map", false);  // HH_260127 allow loading maps with OSM validation warnings
    // HH_260109 default to fused localization pose for lanelet-guided cost grid.
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/planning/global_path");  // HH_260103 focus cost along planned path
    republish_period_ = declare_parameter<double>("republish_period", 1.0);  // HH_260103 RViz toggle refresh
    output_topic_ = declare_parameter<std::string>("output_topic", "/map/lanelet_cost_grid");  // HH_260123 allow multiple cost grids

    // HH_260109 Publish lanelet cost grid under /map prefix.
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local());
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&LaneletCostGridNode::onParamChange, this, std::placeholders::_1));
    republish_timer_ = create_wall_timer(
      std::chrono::duration<double>(republish_period_),
      std::bind(&LaneletCostGridNode::onRepublishTimer, this));

    if (map_path_.empty() || !std::filesystem::exists(map_path_)) {
      RCLCPP_ERROR(get_logger(), "map_path invalid or missing: '%s'", map_path_.c_str());
      return;
    }

    try {
      // HH_260101 load with UTM projector using provided geodetic origin
      lanelet::projection::UtmProjector projector(
        lanelet::Origin(lanelet::GPSPoint{origin_lat_, origin_lon_, origin_alt_}));
      lanelet::ErrorMessages errors;
      map_ = lanelet::load(map_path_, projector, &errors);
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

    // HH_251231 subscribe to robot pose and build grid around robot
    using std::placeholders::_1;
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LaneletCostGridNode::onPose, this, _1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1),
      std::bind(&LaneletCostGridNode::onPath, this, _1));
    RCLCPP_DEBUG(get_logger(), "waiting pose on %s to build cost grid", pose_topic_.c_str());

    // HH_260125 TF listener to align incoming poses/paths to map_frame_ if frame_id differs.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void onRepublishTimer()
  {
    if (!last_grid_.data.empty()) {
      last_grid_.header.stamp = now();
      grid_pub_->publish(last_grid_);
    }
  }

private:
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

  void onPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_in_map;
    if (!transformToMap(*msg, pose_in_map)) {
      return;
    }
    current_pose_ = pose_in_map;
    has_pose_ = true;
    buildGrid();
  }

  void onPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      return;
    }
    nav_msgs::msg::Path path_map;
    path_map.header = msg->header;
    path_map.header.frame_id = frame_id_;
    path_map.poses.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      geometry_msgs::msg::PoseStamped ps_map;
      if (!transformToMap(ps, ps_map)) {
        continue;
      }
      path_map.poses.push_back(ps_map);
    }
    if (path_map.poses.size() < 2) {
      return;
    }
    path_ = path_map;
    path_received_ = true;
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
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
      } else if (p.get_name() == "path_use_lanelet_mask") {
        path_use_lanelet_mask_ = p.as_bool();
      } else if (p.get_name() == "path_lane_base_value") {
        path_lane_base_value_ = p.as_int();
      } else if (p.get_name() == "backward_penalty") {
        backward_penalty_ = p.as_int();
      } else if (p.get_name() == "direction_penalty") {
        direction_penalty_ = p.as_int();
      } else if (p.get_name() == "grid_yaw") {
        grid_yaw_ = p.as_double();
      } else if (p.get_name() == "cost_mode") {
        cost_mode_ = p.as_string();
      } else if (p.get_name() == "boundary_half_width") {
        boundary_half_width_ = p.as_double();
      } else if (p.get_name() == "outside_value") {
        outside_value_ = p.as_int();
      } else if (p.get_name() == "use_map_bbox") {
        use_map_bbox_ = p.as_bool();
      }
    }
    // regenerate grid if pose already exists
    if (has_pose_) {
      buildGrid();
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    res.reason = "updated";
    return res;
  }

  void buildGrid()
  {
    if (!map_) return;
    if (!has_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "pose not received yet; skip grid publish");
      return;
    }
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.info.resolution = resolution_;

    double yaw = grid_yaw_;
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
        const double margin = centerline_half_width_ + gradient_range_;
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
      const double margin = centerline_half_width_ + gradient_range_;
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
    grid.info.origin.position.x =
      std::floor(raw_origin_x / resolution_ + 0.5) * resolution_;
    grid.info.origin.position.y =
      std::floor(raw_origin_y / resolution_ + 0.5) * resolution_;
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
      default_cell = free_value_;
    }
    grid.data.assign(grid_w * grid_h, default_cell);
    // HH_260101 keep origin for rasterization
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;

    // HH_260101 fill free cells along lanelet centerlines only (narrow corridor)
    const double robot_yaw = yawFromPose(current_pose_);
    const double robot_cos = std::cos(robot_yaw);
    const double robot_sin = std::sin(robot_yaw);

    if (cost_mode_ == "path") {
      if (path_use_lanelet_mask_) {
        const int lane_cost = std::clamp(path_lane_base_value_, free_value_, lethal_value_);
        for (const auto & ll : map_->laneletLayer) {
          fillLaneletArea(ll, lane_cost, grid);
        }
      }
      if (path_received_ && path_.poses.size() > 1) {
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
          fillPathStrip(strip, p0_msg, p1_msg, heading_dot < 0.0, robot_cos, robot_sin, grid);
        }
      }
    } else if (cost_mode_ == "bounds") {
      for (const auto & ll : map_->laneletLayer) {
        const auto & left = ll.leftBound();
        const auto & right = ll.rightBound();
        if (left.size() >= 2) {
          for (size_t i = 0; i + 1 < left.size(); ++i) {
            fillBoundaryStrip(left[i], left[i + 1], grid);
          }
        }
        if (right.size() >= 2) {
          for (size_t i = 0; i + 1 < right.size(); ++i) {
            fillBoundaryStrip(right[i], right[i + 1], grid);
          }
        }
      }
    } else {  // centerline (default)
      for (const auto & ll : map_->laneletLayer) {
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
          fillPolygon(
            strip, heading_dot < 0.0, robot_cos, robot_sin, grid,
            centerline_use_distance_gradient_);
        }
      }
    }

    grid.header.stamp = now();
    last_grid_ = grid;
    grid_pub_->publish(grid);
    RCLCPP_DEBUG(get_logger(),
      "published lanelet cost grid (%ux%u, res=%.2f) frame=%s",
      grid.info.width, grid.info.height, grid.info.resolution, grid.header.frame_id.c_str());
  }

  void fillPolygon(
    const lanelet::BasicPolygon2d & poly, bool opposite_heading,
    double robot_cos, double robot_sin,
    nav_msgs::msg::OccupancyGrid & grid,
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
        lanelet::BasicPoint2d p(wx, wy);
        if (lanelet::geometry::within(p, poly)) {
          int cost = free_value_;
          if (use_distance_gradient) {
            const double dist =
              std::hypot(wx - current_pose_.pose.position.x, wy - current_pose_.pose.position.y);
            const double norm = gradient_range_ > 1e-3 ? dist / gradient_range_ : 0.0;
            cost = std::clamp(
              static_cast<int>(norm * lethal_value_), free_value_, lethal_value_);
          }
          if (opposite_heading) {
            cost = std::clamp(cost + direction_penalty_, free_value_, lethal_value_);
          }
          // HH_260102 penalize cells behind robot heading to discourage reverse driving
          const double vx = wx - current_pose_.pose.position.x;
          const double vy = wy - current_pose_.pose.position.y;
          const double forward_dot = vx * robot_cos + vy * robot_sin;
          if (forward_dot < 0.0) {
            cost = std::clamp(cost + backward_penalty_, free_value_, lethal_value_);
          }
          grid.data[iy * grid_w + ix] = cost;
        }
      }
    }
  }

  void fillBoundaryStrip(
    const lanelet::ConstPoint3d & p0,
    const lanelet::ConstPoint3d & p1,
    nav_msgs::msg::OccupancyGrid & grid)
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
    fillPolygonConst(strip, lethal_value_, grid);
  }

  void fillLaneletArea(
    const lanelet::ConstLanelet & ll, int value,
    nav_msgs::msg::OccupancyGrid & grid)
  {
    const auto & left = ll.leftBound();
    const auto & right = ll.rightBound();
    if (left.size() < 2 || right.size() < 2) {
      return;
    }
    lanelet::BasicPolygon2d poly;
    poly.reserve(left.size() + right.size());
    for (const auto & p : left) {
      poly.emplace_back(p.x(), p.y());
    }
    for (size_t i = right.size(); i > 0; --i) {
      const auto & p = right[i - 1];
      poly.emplace_back(p.x(), p.y());
    }
    fillPolygonConst(poly, value, grid);
  }

  void fillPathStrip(
    const lanelet::BasicPolygon2d & poly,
    const geometry_msgs::msg::Point & p0,
    const geometry_msgs::msg::Point & p1,
    bool opposite_heading,
    double robot_cos, double robot_sin,
    nav_msgs::msg::OccupancyGrid & grid)
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
    const double half_width = std::max(1e-3, centerline_half_width_);

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double wx = origin_x_ + (ix + 0.5) * resolution_;
        const double wy = origin_y_ + (iy + 0.5) * resolution_;
        lanelet::BasicPoint2d p(wx, wy);
        if (!lanelet::geometry::within(p, poly)) {
          continue;
        }
        double dist = 0.0;
        if (vlen2 > 1e-6) {
          const double t = std::clamp(((wx - p0.x) * vx + (wy - p0.y) * vy) / vlen2, 0.0, 1.0);
          const double px = p0.x + t * vx;
          const double py = p0.y + t * vy;
          dist = std::hypot(wx - px, wy - py);
        }
        const double norm = std::clamp(dist / half_width, 0.0, 1.0);
        int cost = static_cast<int>(free_value_ + norm * (lethal_value_ - free_value_));
        if (opposite_heading) {
          cost = std::clamp(cost + direction_penalty_, free_value_, lethal_value_);
        }
        const double fvx = wx - current_pose_.pose.position.x;
        const double fvy = wy - current_pose_.pose.position.y;
        const double forward_dot = fvx * robot_cos + fvy * robot_sin;
        if (forward_dot < 0.0) {
          cost = std::clamp(cost + backward_penalty_, free_value_, lethal_value_);
        }
        grid.data[iy * grid_w + ix] = cost;
      }
    }
  }

  void fillPolygonConst(
    const lanelet::BasicPolygon2d & poly, int value,
    nav_msgs::msg::OccupancyGrid & grid)
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
        lanelet::BasicPoint2d p(wx, wy);
        if (lanelet::geometry::within(p, poly)) {
          grid.data[iy * grid_w + ix] = value;
        }
      }
    }
  }

  double yawFromPose(const geometry_msgs::msg::PoseStamped & pose) const
  {
    const auto & q = pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // HH_260125 Transform helper: ensure all poses are in map frame.
  bool transformToMap(
    const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out)
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
  double origin_lat_;
  double origin_lon_;
  double origin_alt_;
  int free_value_;
  int lethal_value_;
  int outside_value_;
  double gradient_range_;
  int direction_penalty_;
  int backward_penalty_;
  bool centerline_use_distance_gradient_;
  bool path_use_lanelet_mask_;
  int path_lane_base_value_;
  double grid_yaw_;
  std::string cost_mode_;
  bool use_path_bbox_;
  bool lock_window_;
  bool use_map_bbox_;
  bool ignore_invalid_map_;
  std::string pose_topic_;
  std::string path_topic_;
  double republish_period_;
  std::string output_topic_;

  bool map_bounds_valid_{false};
  double map_min_x_{0.0};
  double map_min_y_{0.0};
  double map_max_x_{0.0};
  double map_max_y_{0.0};

  lanelet::LaneletMapPtr map_;
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Path path_;
  bool has_pose_{false};
  bool path_received_{false};
  nav_msgs::msg::OccupancyGrid last_grid_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneletCostGridNode>());
  rclcpp::shutdown();
  return 0;
}
