#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

// HH_260720 - Use generated CAMROD localization/planning pose contracts.
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_planning_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/avg_quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <rclcpp/rclcpp.hpp>

#include "camrod_map/custom_regulatory_elements.hpp"  // HH_260114 Register speed_bump rule.

namespace
{
std::string normalizeModeToken(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

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
avg_msgs::msg::AvgQuaternion yawToQuat(double yaw)
{
  avg_msgs::msg::AvgQuaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

double yawFromQuat(const avg_msgs::msg::AvgQuaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class CenterlineSnapperNode : public rclcpp::Node
{
public:
  using AvgPlanningMsgs = avg_msgs::msg::AvgPlanningMsgs;
  using ModuleState = avg_msgs::msg::ModuleState;

  CenterlineSnapperNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("centerline_snapper")
  {
    // Parameters
    cfg_.map_path = declare_parameter<std::string>("map_path", "");
    cfg_.offset_lat = declare_parameter<double>("offset_lat", 0.0);
    cfg_.offset_lon = declare_parameter<double>("offset_lon", 0.0);
    cfg_.offset_alt = declare_parameter<double>("offset_alt", 0.0);
    // HH_260109 Default to fused localization pose and publish lanelet constraint pose.
    input_pose_topic_ = declare_parameter<std::string>("input_pose_topic", "/localization/pose");
    input_pose_cov_topic_ =
      declare_parameter<std::string>("input_pose_cov_topic", "/localization/pose_with_covariance");
    output_pose_topic_ = declare_parameter<std::string>(
      "output_pose_topic", "/localization/centerline_pose");
    // HH_260317-00:00 Publish ROS-native lanelet pose for TF/Nav2 helpers.
    output_pose_topic_ros_ = declare_parameter<std::string>(
      "output_pose_topic_ros", "/planning/lanelet_pose_ros");
    output_pose_cov_topic_ = declare_parameter<std::string>(
      "output_pose_cov_topic", output_pose_topic_ + "_with_covariance");
    publish_planning_status_ = declare_parameter<bool>("publish_planning_status", false);
    planning_status_topic_ =
      declare_parameter<std::string>("planning_status_topic", "/planning/status");
    max_search_radius_ = declare_parameter<double>("max_search_radius", 30.0);
    longitudinal_stddev_ = declare_parameter<double>("longitudinal_stddev", 0.5);
    lateral_stddev_ = declare_parameter<double>("lateral_stddev", 0.3);
    yaw_stddev_ = declare_parameter<double>("yaw_stddev", 0.2);
    // HH_260629: Prefer yaw-aligned centerlines only while localization yaw covariance is healthy.
    heading_filter_enable_ = declare_parameter<bool>("heading_filter_enable", true);
    max_heading_error_deg_ = declare_parameter<double>("max_heading_error_deg", 100.0);
    heading_filter_max_yaw_variance_ =
      declare_parameter<double>("heading_filter_max_yaw_variance", 1.0);
    // HH_260526: Replace use_map_z/flatten_to_ground booleans with one explicit mode.
    // HH_260623 - "ground" means the 2D planning plane (Z=0), not raw OSM median altitude.
    // centerline_z_mode options: input | map | ground.
    centerline_z_mode_ = normalizeModeToken(
      declare_parameter<std::string>("centerline_z_mode", "ground"));
    map_z_offset_ = declare_parameter<double>("map_z_offset", 0.0);    // HH_260114 Extra z offset applied to map elevation.
    // HH_260413: Throttle nearest-centerline search under high-rate localization streams.
    min_update_period_s_ = declare_parameter<double>("min_update_period_s", 0.05);
    min_input_displacement_m_ = declare_parameter<double>("min_input_displacement_m", 0.05);
    if (
      centerline_z_mode_ != "input" &&
      centerline_z_mode_ != "map" &&
      centerline_z_mode_ != "ground")
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid centerline_z_mode='%s'. Falling back to 'ground'.",
        centerline_z_mode_.c_str());
      centerline_z_mode_ = "ground";
    }

    if (!loadMap()) {
      RCLCPP_FATAL(get_logger(), "centerline snapper: failed to load map. exiting.");
      rclcpp::shutdown();
      return;
    }

    if (output_pose_cov_topic_ == output_pose_topic_) {
      output_pose_cov_topic_ = output_pose_topic_ + "_with_covariance";
      RCLCPP_WARN(
        get_logger(),
        "output_pose_cov_topic matched output_pose_topic. Using '%s' for covariance output instead.",
        output_pose_cov_topic_.c_str());
    }

    // HH_260720 - Publish the canonical lanelet pose as a generated CAMROD message.
    pub_pose_ = create_publisher<avg_msgs::msg::AvgPoseStamped>(
      output_pose_topic_, rclcpp::QoS(10));
    pub_pose_ros_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_pose_topic_ros_, rclcpp::QoS(10));
    pub_pose_cov_ = create_publisher<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      output_pose_cov_topic_, rclcpp::QoS(10));
    if (publish_planning_status_) {
      pub_avg_planning_ = create_publisher<AvgPlanningMsgs>(
        planning_status_topic_, rclcpp::QoS(10));
    }
    // HH_260305-00:00 Use reliable/latest-only pose QoS.
    // Best-effort drop under load makes snapped pose lag and causes downstream local-path jitter.
    sub_pose_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      input_pose_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&CenterlineSnapperNode::onPose, this, std::placeholders::_1));
    sub_pose_cov_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      input_pose_cov_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&CenterlineSnapperNode::onPoseCovariance, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "centerline_snapper ready: map=%s input=%s output=%s output_cov=%s",
      cfg_.map_path.c_str(), input_pose_topic_.c_str(), output_pose_topic_.c_str(),
      output_pose_cov_topic_.c_str());
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
    RCLCPP_DEBUG(
      get_logger(), "Loaded lanelet map: lanelets=%zu linestrings=%zu",
      map_->laneletLayer.size(), map_->lineStringLayer.size());
    map_ground_z_ = computeGroundZ(*map_);
    RCLCPP_DEBUG(get_logger(), "map ground z (median)=%.3f", map_ground_z_);
    return true;
  }

  // Handles the `onPose` callback.
  void onPose(const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr msg)
  {
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;
    const rclcpp::Time stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
      get_clock()->now() :
      rclcpp::Time(msg->header.stamp);

    // HH_260413: Skip map projection when both time and displacement thresholds are not met.
    if (has_last_publish_) {
      const double dt = (stamp - last_publish_stamp_).seconds();
      const double moved = std::hypot(px - last_input_x_, py - last_input_y_);
      const bool use_period = min_update_period_s_ > 0.0;
      const bool use_distance = min_input_displacement_m_ > 0.0;
      const bool skip_by_period = use_period && dt < min_update_period_s_;
      const bool skip_by_distance = use_distance && moved < min_input_displacement_m_;
      if ((use_period || use_distance) &&
        (!use_period || skip_by_period) &&
        (!use_distance || skip_by_distance))
      {
        return;
      }
    }

    const auto nearest = findNearestCenterline(px, py, yawFromQuat(msg->pose.orientation));
    if (!nearest.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No centerline within search radius %.1f m",
        max_search_radius_);
      return;
    }
    avg_msgs::msg::AvgPoseStamped out_pose;
    out_pose.header = msg->header;
    out_pose.pose.position.x = nearest.nearest_point.x();
    out_pose.pose.position.y = nearest.nearest_point.y();
    // HH_260526: Keep z-selection explicit by mode instead of coupled booleans.
    double snapped_z = pz;
    if (centerline_z_mode_ == "map") {
      snapped_z = nearest.nearest_point.z() + map_z_offset_;
    } else if (centerline_z_mode_ == "ground") {
      snapped_z = map_z_offset_;
    }
    out_pose.pose.position.z = snapped_z;
    out_pose.pose.orientation = yawToQuat(nearest.heading);

    avg_msgs::msg::AvgPoseWithCovarianceStamped out_cov;
    out_cov.header = out_pose.header;
    out_cov.pose.pose = out_pose.pose;

    // Fill covariance: longitudinal, lateral, yaw
    for (auto & c : out_cov.pose.covariance) {
      c = 0.0;
    }
    out_cov.pose.covariance[0] = longitudinal_stddev_ * longitudinal_stddev_;
    out_cov.pose.covariance[7] = lateral_stddev_ * lateral_stddev_;
    out_cov.pose.covariance[14] = 9999.0;  // z not observed
    out_cov.pose.covariance[21] = 9999.0;
    out_cov.pose.covariance[28] = 9999.0;
    out_cov.pose.covariance[35] = yaw_stddev_ * yaw_stddev_;

    pub_pose_->publish(out_pose);
    publishRosPose(out_pose);
    pub_pose_cov_->publish(out_cov);
    // Runtime trace for start-pose validation in planning.
    // Keep this at DEBUG level to avoid high-frequency log noise in normal runs.
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "centerline_snapper: in=(%.2f, %.2f) out=(%.2f, %.2f) dist=%.2f",
      px, py, out_pose.pose.position.x, out_pose.pose.position.y, std::sqrt(nearest.sq_dist));
    publishAvgPlanning(out_pose);
    last_publish_stamp_ = stamp;
    last_input_x_ = px;
    last_input_y_ = py;
    has_last_publish_ = true;
  }

  void onPoseCovariance(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    latest_yaw_variance_ = msg->pose.covariance[35];
  }

  // Publishes `RosPose` output.
  void publishRosPose(const avg_msgs::msg::AvgPoseStamped & pose)
  {
    if (!pub_pose_ros_) {
      return;
    }
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = pose.header.stamp;
    out.header.frame_id = pose.header.frame_id;
    out.pose.position.x = pose.pose.position.x;
    out.pose.position.y = pose.pose.position.y;
    out.pose.position.z = pose.pose.position.z;
    out.pose.orientation.x = pose.pose.orientation.x;
    out.pose.orientation.y = pose.pose.orientation.y;
    out.pose.orientation.z = pose.pose.orientation.z;
    out.pose.orientation.w = pose.pose.orientation.w;
    pub_pose_ros_->publish(out);
  }

  // Publishes `AvgPlanning` output.
  void publishAvgPlanning(const avg_msgs::msg::AvgPoseStamped & lanelet_pose)
  {
    if (!publish_planning_status_ || !pub_avg_planning_) {
      return;
    }
    AvgPlanningMsgs msg;
    msg.stamp = now();
    msg.state.stamp = msg.stamp;
    msg.state.module_name = "planning";
    msg.state.level = ModuleState::OK;
    msg.state.message = "centerline_snapper";
    // HH_260720 - The lanelet pose is already a generated CAMROD message.
    msg.lanelet_pose = lanelet_pose;
    pub_avg_planning_->publish(msg);
  }

  // Implements `findNearestCenterline` behavior.
  NearestResult findNearestCenterline(double x, double y, double pose_yaw) const
  {
    NearestResult best;
    NearestResult best_heading_aligned;
    const double max_sq = max_search_radius_ * max_search_radius_;
    const double max_heading_error_rad = max_heading_error_deg_ * M_PI / 180.0;
    const bool heading_filter_allowed =
      heading_filter_enable_ &&
      std::isfinite(latest_yaw_variance_) &&
      latest_yaw_variance_ > 0.0 &&
      latest_yaw_variance_ <= heading_filter_max_yaw_variance_;
    for (const auto & ll : map_->laneletLayer) {
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
        const double heading = std::atan2(vy, vx);
        if (dist2 < best.sq_dist && dist2 < max_sq) {
          best.sq_dist = dist2;
          best.valid = true;
          const double proj_z = p0.z() + t * (p1.z() - p0.z());  // HH_260114 Interpolate z along segment.
          best.nearest_point = lanelet::Point3d(lanelet::InvalId, proj_x, proj_y, proj_z);
          best.heading = heading;
        }
        const double heading_error = std::abs(normalizeAngle(heading - pose_yaw));
        if (heading_filter_allowed && heading_error <= max_heading_error_rad &&
          dist2 < best_heading_aligned.sq_dist && dist2 < max_sq)
        {
          best_heading_aligned.sq_dist = dist2;
          best_heading_aligned.valid = true;
          const double proj_z = p0.z() + t * (p1.z() - p0.z());
          best_heading_aligned.nearest_point =
            lanelet::Point3d(lanelet::InvalId, proj_x, proj_y, proj_z);
          best_heading_aligned.heading = heading;
        }
      }
    }
    if (heading_filter_allowed && best_heading_aligned.valid) {
      return best_heading_aligned;
    }
    return best;
  }

  LoaderConfig cfg_;
  lanelet::LaneletMapPtr map_;

  std::string input_pose_topic_;
  std::string input_pose_cov_topic_;
  std::string output_pose_topic_;
  std::string output_pose_topic_ros_;
  std::string output_pose_cov_topic_;
  std::string planning_status_topic_;
  double max_search_radius_{30.0};
  double longitudinal_stddev_{0.5};
  double lateral_stddev_{0.3};
  double yaw_stddev_{0.2};
  bool heading_filter_enable_{true};
  double max_heading_error_deg_{100.0};
  double heading_filter_max_yaw_variance_{1.0};
  double latest_yaw_variance_{std::numeric_limits<double>::infinity()};
  std::string centerline_z_mode_{"ground"};  // HH_260623 - Default to 2D planning plane.
  double map_z_offset_{0.0};
  double min_update_period_s_{0.05};
  double min_input_displacement_m_{0.05};
  rclcpp::Time last_publish_stamp_{0, 0, RCL_ROS_TIME};
  double last_input_x_{0.0};
  double last_input_y_{0.0};
  bool has_last_publish_{false};
  double map_ground_z_{0.0};

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

  rclcpp::Publisher<avg_msgs::msg::AvgPoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ros_;
  rclcpp::Publisher<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<AvgPlanningMsgs>::SharedPtr pub_avg_planning_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr sub_pose_cov_;
  bool publish_planning_status_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineSnapperNode>());
  rclcpp::shutdown();
  return 0;
}
