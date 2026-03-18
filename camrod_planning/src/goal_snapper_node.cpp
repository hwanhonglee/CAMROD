#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <avg_msgs/msg/avg_planning_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>
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
  using ModuleHealth = avg_msgs::msg::ModuleHealth;

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
    // HH_260317-00:00 Publish ROS-native snapped goal for Nav2 topic compatibility.
    output_goal_topic_ros_ = declare_parameter<std::string>(
      "output_goal_topic_ros", "/planning/goal_pose_snapped_ros");
    publish_planning_diagnostic_ = declare_parameter<bool>("publish_planning_diagnostic", false);
    planning_diagnostic_topic_ =
      declare_parameter<std::string>("planning_diagnostic_topic", "/planning/diagnostic");
    max_search_radius_ = declare_parameter<double>("max_search_radius", 30.0);
    require_lanelet_containment_ = declare_parameter<bool>("require_lanelet_containment", true);
    fallback_uncontained_ = declare_parameter<bool>("fallback_uncontained", true);
    use_map_z_ = declare_parameter<bool>("use_map_z", true);
    flatten_to_ground_ = declare_parameter<bool>("flatten_to_ground", true);
    map_z_offset_ = declare_parameter<double>("map_z_offset", 0.0);

    if (!loadMap()) {
      RCLCPP_FATAL(get_logger(), "goal snapper: failed to load map. exiting.");
      rclcpp::shutdown();
      return;
    }

    pub_goal_ = create_publisher<avg_msgs::msg::PoseStamped>(
      output_goal_topic_, rclcpp::QoS(10));
    pub_goal_ros_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_goal_topic_ros_, rclcpp::QoS(10));
    if (publish_planning_diagnostic_) {
      pub_avg_planning_ = create_publisher<AvgPlanningMsgs>(
        planning_diagnostic_topic_, rclcpp::QoS(10));
    }
    sub_goal_ = create_subscription<avg_msgs::msg::PoseStamped>(
      input_goal_topic_, rclcpp::QoS(10),
      std::bind(&GoalSnapperNode::onGoal, this, std::placeholders::_1));
    sub_goal_ros_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_goal_topic_, rclcpp::QoS(10),
      std::bind(&GoalSnapperNode::onGoalRos, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "goal_snapper ready: map=%s input=%s output(avg)=%s output(ros)=%s",
      cfg_.map_path.c_str(), input_goal_topic_.c_str(),
      output_goal_topic_.c_str(), output_goal_topic_ros_.c_str());
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

  // Handles the `onGoal` callback.
  void onGoal(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;

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
    // HH_260316-00:00 Keep explicit runtime trace for path-failure diagnosis.
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
    // HH_260317-00:00 Accept RViz 2D Goal Pose directly (geometry_msgs).
    // Internal planning helpers still consume avg_msgs output_goal_topic_.
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;

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
    if (!publish_planning_diagnostic_ || !pub_avg_planning_) {
      return;
    }
    AvgPlanningMsgs msg;
    msg.stamp = now();
    msg.health.stamp = msg.stamp;
    msg.health.module_name = "planning";
    msg.health.level = ModuleHealth::OK;
    msg.health.message = "goal_snapper";
    msg.goal_pose = goal_pose;
    pub_avg_planning_->publish(msg);
  }

  // Implements `findNearestCenterline` behavior.
  NearestResult findNearestCenterline(
    double x, double y, bool require_lanelet_containment) const
  {
    NearestResult best;
    const double max_sq = max_search_radius_ * max_search_radius_;
    for (const auto & ll : map_->laneletLayer) {
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
  std::string planning_diagnostic_topic_;
  double max_search_radius_{30.0};
  bool require_lanelet_containment_{true};
  bool fallback_uncontained_{true};
  bool use_map_z_{true};
  bool flatten_to_ground_{true};
  double map_z_offset_{0.0};
  double map_ground_z_{0.0};

  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_ros_;
  rclcpp::Publisher<AvgPlanningMsgs>::SharedPtr pub_avg_planning_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_ros_;
  bool publish_planning_diagnostic_{false};
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSnapperNode>());
  rclcpp::shutdown();
  return 0;
}
