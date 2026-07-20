#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

// HH_260720 - Separate generated CAMROD data contracts from RViz and TF ROS boundaries.
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_platform_msgs.hpp>
#include <avg_msgs/msg/avg_point32.hpp>
#include <avg_msgs/msg/avg_polygon_stamped.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "camrod_sensor_kit/robot_params.hpp"  // HH_260109 renamed package
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace camrod
{
namespace
{
std::string normalizeModeToken(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

// HH_260114 Keep a single label scale so every TF text marker is consistent across sensors.
constexpr double kLabelScale = 0.22;

// Builds an avg_msgs color object using RGBA components in [0.0, 1.0].
std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// Builds a 3D point helper object.
geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

// Converts roll/pitch/yaw (rad) to quaternion.
geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion quat;
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();
  return quat;
}

// Normalizes yaw to [-pi, pi].
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

}  // namespace

struct PoseRPY
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

class RobotVisualizationNode : public rclcpp::Node
{
public:
  // HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

  RobotVisualizationNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : Node("robot_visualization")
  {
    params_ = loadRobotParams(this);
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    // HH_260304-00:00 // Platform ownership uses /platform/robot/* topics by default.
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/platform/robot/markers");
    boundary_topic_ = declare_parameter<std::string>(
      "boundary_topic", "/platform/robot/planning_boundary");
    publish_platform_status_ = declare_parameter<bool>("publish_platform_status", false);
    platform_status_topic_ =
      declare_parameter<std::string>("platform_status_topic", "/platform/status");
    // HH_260304-00:00 // Visualizer should not publish TF unless explicitly requested.
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    // HH_260327: Platform marker follows localization pose by default.
    // GNSS is optional fallback when localization pose is stale/missing.
    localization_pose_topic_ = declare_parameter<std::string>(
      "localization_pose_topic", "/localization/pose");
    gnss_pose_topic_ = declare_parameter<std::string>(
      "gnss_pose_topic", "/sensing/gnss/pose");
    // HH_260526: Replace use_gnss_fallback toggle with explicit pose source mode.
    // pose_source_mode options:
    //   localization_only
    //   localization_with_gnss_fallback
    pose_source_mode_ = normalizeModeToken(
      declare_parameter<std::string>("pose_source_mode", "localization_with_gnss_fallback"));
    localization_pose_timeout_s_ = declare_parameter<double>(
      "localization_pose_timeout_s", 1.0);
    // HH_260409: Optional heading offset for platform visualization alignment.
    // Negative value rotates clockwise in ROS yaw convention.
    heading_yaw_offset_deg_ = declare_parameter<double>("heading_yaw_offset_deg", 0.0);
    heading_yaw_offset_rad_ = heading_yaw_offset_deg_ * M_PI / 180.0;
    // HH_260506: Keep RViz marker list compact and optionally hide body cube marker.
    show_chassis_marker_ = declare_parameter<bool>("show_chassis_marker", true);
    group_robot_marker_namespaces_ = declare_parameter<bool>(
      "group_robot_marker_namespaces", true);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
    // HH_260618: Apply publish_rate_hz to both timer and pose-callback driven
    // marker updates. Previously localization callbacks could drive markers at
    // the localization rate, ignoring the configured visualization rate.
    marker_publish_period_s_ = publish_rate_hz > 0.0 ? 1.0 / publish_rate_hz : 1.0;
    body_scale_factor_ = declare_parameter<double>("body_scale_factor", 1.0);
    // HH_260623 - Default visualization boundary margin follows measured robot_params planning margin.
    planning_boundary_margin_ = declare_parameter<double>(
      "planning_boundary_margin", params_.planning_margin);
    ground_z_offset_ = declare_parameter<double>("ground_z_offset", 0.0);
    range_ring_radii_ = declare_parameter<std::vector<double>>(
      "range_ring_radii", std::vector<double>{2.0, 4.0, 6.0, 8.0});
    // HH_260526: Replace use_map_ground_z toggle with explicit source mode.
    // ground_z_source options: fixed_offset | lanelet_map.
    ground_z_source_ = normalizeModeToken(
      declare_parameter<std::string>("ground_z_source", "lanelet_map"));
    // HH_260623 - Visualization is 2D-ground anchored by default; real pose altitude can be enabled explicitly.
    // pose_z_source options: ground | pose.
    pose_z_source_ = normalizeModeToken(
      declare_parameter<std::string>("pose_z_source", "ground"));
    base_pose_.x = declare_parameter<double>("base_pose.x", 0.0);
    base_pose_.y = declare_parameter<double>("base_pose.y", 0.0);
    base_pose_.z = declare_parameter<double>("base_pose.z", ground_z_offset_);
    base_pose_.roll = declare_parameter<double>("base_pose.roll", 0.0);
    base_pose_.pitch = declare_parameter<double>("base_pose.pitch", 0.0);
    base_pose_.yaw = declare_parameter<double>("base_pose.yaw", 0.0);
    // HH_260409: Keep startup base yaw aligned with configured heading offset.
    base_pose_.yaw = normalizeAngle(base_pose_.yaw + heading_yaw_offset_rad_);
    if (
      pose_source_mode_ != "localization_only" &&
      pose_source_mode_ != "localization_with_gnss_fallback")
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid pose_source_mode='%s'. Falling back to 'localization_with_gnss_fallback'.",
        pose_source_mode_.c_str());
      pose_source_mode_ = "localization_with_gnss_fallback";
    }
    if (ground_z_source_ != "fixed_offset" && ground_z_source_ != "lanelet_map") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid ground_z_source='%s'. Falling back to 'lanelet_map'.",
        ground_z_source_.c_str());
      ground_z_source_ = "lanelet_map";
    }
    if (pose_z_source_ != "ground" && pose_z_source_ != "pose") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid pose_z_source='%s'. Falling back to 'ground'.",
        pose_z_source_.c_str());
      pose_z_source_ = "ground";
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos);
    // HH_260720 - Publish the planning boundary as a generated CAMROD polygon.
    boundary_pub_ = create_publisher<avg_msgs::msg::AvgPolygonStamped>(boundary_topic_, qos);
    if (publish_platform_status_) {
      avg_platform_pub_ = create_publisher<avg_msgs::msg::AvgPlatformMsgs>(platform_status_topic_, qos);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    // HH_260720 - Keep RViz initial pose as an explicit external ROS boundary.
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&RobotVisualizationNode::onInitialPoseRos, this, std::placeholders::_1));
    localization_pose_sub_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      localization_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&RobotVisualizationNode::onLocalizationPose, this, std::placeholders::_1));
    gnss_sub_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      gnss_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&RobotVisualizationNode::onGnssPose, this, std::placeholders::_1));
    if (ground_z_source_ == "lanelet_map") {
      map_marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/map/markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&RobotVisualizationNode::onMapMarkers, this, std::placeholders::_1));
    }

    publishBaseTransform();
    publishMarkers(true);

    using namespace std::chrono_literals;
    const auto period = marker_publish_period_s_ > 0.0
      ? std::chrono::duration<double>(marker_publish_period_s_)
      : std::chrono::seconds(1);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publishMarkers(false);
      });

    // HH_260304-00:00 // Keep startup logs quiet by default.
    RCLCPP_DEBUG(
      get_logger(),
      "robot visualization ready. Marker topic '%s', base frame '%s', map frame '%s'.",
      marker_topic_.c_str(), base_frame_id_.c_str(), map_frame_id_.c_str());
  }

private:
  // Broadcasts map->base transform when TF publishing is enabled.
  void publishBaseTransform()
  {
    if (!publish_tf_ || !tf_broadcaster_) {
      return;
    }
    geometry_msgs::msg::TransformStamped base_tf;
    base_tf.header.stamp = this->now();
    base_tf.header.frame_id = map_frame_id_;
    base_tf.child_frame_id = base_frame_id_;
    base_tf.transform.translation.x = base_pose_.x;
    base_tf.transform.translation.y = base_pose_.y;
    base_tf.transform.translation.z = base_pose_.z;
    base_tf.transform.rotation = quaternionFromRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw);
    tf_broadcaster_->sendTransform(base_tf);
  }

  // Rebuilds and publishes all robot/sensor visualization markers and planning boundary polygon.
  void publishMarkers(bool force = false)
  {
    const auto now = this->get_clock()->now();
    if (!force && last_marker_publish_time_.nanoseconds() > 0 && marker_publish_period_s_ > 0.0) {
      const double age_s = (now - last_marker_publish_time_).seconds();
      if (age_s < marker_publish_period_s_) {
        return;
      }
    }
    publishBaseTransform();
    visualization_msgs::msg::MarkerArray markers;
    int32_t marker_id = 0;
    tf2::Quaternion base_tf;
    base_tf.setRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw);
    base_tf.normalize();
    tf2::Matrix3x3 base_rot(base_tf);
    const geometry_msgs::msg::Quaternion base_orientation = tf2::toMsg(base_tf);
    const geometry_msgs::msg::Point base_translation =
      makePoint(base_pose_.x, base_pose_.y, base_pose_.z);
    const std::string base_label_ns = group_robot_marker_namespaces_
      ? "robot/base"
      : "robot_base_link";
    const std::string base_axes_ns = base_label_ns;
    const std::string body_ns = group_robot_marker_namespaces_
      ? "robot/chassis"
      : "robot_body";
    const std::string footprint_ns = group_robot_marker_namespaces_
      ? "robot/chassis"
      : "robot_footprint";
    const std::string boundary_ns = group_robot_marker_namespaces_
      ? "robot/chassis"
      : "robot_planning_boundary";
    // HH_260114 Reusable map->robot_base_link transform lambda shared by sensors/bounds/rings.
    const auto transformLocal = [&](double x, double y, double z) {
      const tf2::Vector3 rotated = base_rot * tf2::Vector3(x, y, z);
      return makePoint(
        base_translation.x + rotated.x(),
        base_translation.y + rotated.y(),
        base_translation.z + rotated.z());
    };

    const geometry_msgs::msg::Quaternion identity_orientation = quaternionFromRPY(0.0, 0.0, 0.0);
    markers.markers.emplace_back(
      createAxesMarker(
        "tf/world", marker_id++, makePoint(0.0, 0.0, 0.0),
        identity_orientation, 2.0, now, "world"));
    visualization_msgs::msg::Marker world_label;
    world_label.header.frame_id = "world";
    world_label.header.stamp = now;
    world_label.ns = "tf/world";
    world_label.id = marker_id++;
    world_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    world_label.action = visualization_msgs::msg::Marker::ADD;
    world_label.pose.position = makePoint(0.0, 0.0, 0.6);
    world_label.scale.z = kLabelScale;
    world_label.color = makeColor(0.8f, 0.8f, 0.8f, 0.9f);
    world_label.text = "world";
    markers.markers.emplace_back(world_label);

    markers.markers.emplace_back(
      createAxesMarker(
        "tf/map", marker_id++, makePoint(0.0, 0.0, 0.0),
        identity_orientation, 1.5, now, map_frame_id_));
    visualization_msgs::msg::Marker map_label;
    map_label.header.frame_id = map_frame_id_;
    map_label.header.stamp = now;
    map_label.ns = "tf/map";
    map_label.id = marker_id++;
    map_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    map_label.action = visualization_msgs::msg::Marker::ADD;
    map_label.pose.position = makePoint(0.0, 0.0, 0.5);
    map_label.scale.z = kLabelScale;
    map_label.color = makeColor(0.9f, 0.9f, 0.6f, 0.9f);
    map_label.text = map_frame_id_;
    markers.markers.emplace_back(map_label);

    // HH_260623 - Vehicle body is asymmetric because robot_base_link is the rear-wheel center.
    const double body_front = params_.body_front_extent * body_scale_factor_;
    const double body_rear = params_.body_rear_extent * body_scale_factor_;
    const double body_left = params_.body_left_extent * body_scale_factor_;
    const double body_right = params_.body_right_extent * body_scale_factor_;
    const double body_top_z = params_.body_top_z * body_scale_factor_;
    const double body_bottom_z = params_.body_bottom_z * body_scale_factor_;
    const double body_length = body_front + body_rear;
    const double body_width = body_left + body_right;
    const double body_height = body_top_z - body_bottom_z;
    const double body_center_x = (body_front - body_rear) * 0.5;
    const double body_center_y = (body_left - body_right) * 0.5;
    const double body_center_z = (body_top_z + body_bottom_z) * 0.5;

    // Vehicle bounding box
    if (show_chassis_marker_) {
      visualization_msgs::msg::Marker body_marker;
      body_marker.header.frame_id = map_frame_id_;
      body_marker.header.stamp = now;
      body_marker.ns = body_ns;
      body_marker.id = marker_id++;
      body_marker.type = visualization_msgs::msg::Marker::CUBE;
      body_marker.action = visualization_msgs::msg::Marker::ADD;
      body_marker.pose.position = transformLocal(body_center_x, body_center_y, body_center_z);
      body_marker.pose.orientation = base_orientation;
      body_marker.scale.x = body_length;
      body_marker.scale.y = body_width;
      body_marker.scale.z = body_height;
      body_marker.color = makeColor(0.1f, 0.65f, 0.9f, 0.25f);
      markers.markers.emplace_back(body_marker);
    }

    const double axis_length = 0.8;
    geometry_msgs::msg::Point base_origin = base_translation;
    markers.markers.emplace_back(
      createAxesMarker(
        base_axes_ns, marker_id++, base_origin, base_orientation, axis_length, now,
        map_frame_id_));

    visualization_msgs::msg::Marker base_label;
    base_label.header.frame_id = map_frame_id_;
    base_label.header.stamp = now;
    base_label.ns = base_label_ns;
    base_label.id = marker_id++;
    base_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    base_label.action = visualization_msgs::msg::Marker::ADD;
    base_label.pose.position = transformLocal(0.0, 0.0, body_top_z + 0.3);
    base_label.scale.z = kLabelScale;
    base_label.color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    base_label.text = base_frame_id_;
    markers.markers.emplace_back(base_label);

    // Footprint outline at z = 0
    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header.frame_id = map_frame_id_;
    footprint_marker.header.stamp = now;
    footprint_marker.ns = footprint_ns;
    footprint_marker.id = marker_id++;
    footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.scale.x = 0.02;
    footprint_marker.color = makeColor(0.15f, 0.8f, 0.9f, 0.8f);
    footprint_marker.pose.position = base_translation;
    footprint_marker.pose.orientation = base_orientation;
    footprint_marker.points.push_back(makePoint(body_front, body_left, 0.0));
    footprint_marker.points.push_back(makePoint(body_front, -body_right, 0.0));
    footprint_marker.points.push_back(makePoint(-body_rear, -body_right, 0.0));
    footprint_marker.points.push_back(makePoint(-body_rear, body_left, 0.0));
    footprint_marker.points.push_back(makePoint(body_front, body_left, 0.0));
    markers.markers.emplace_back(footprint_marker);

    visualization_msgs::msg::Marker boundary_marker;
    boundary_marker.header.frame_id = map_frame_id_;
    boundary_marker.header.stamp = now;
    boundary_marker.ns = boundary_ns;
    boundary_marker.id = marker_id++;
    boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    boundary_marker.action = visualization_msgs::msg::Marker::ADD;
    boundary_marker.scale.x = 0.03;
    boundary_marker.color = makeColor(1.0f, 0.85f, 0.0f, 0.95f);
    boundary_marker.pose.position = base_translation;
    boundary_marker.pose.orientation = base_orientation;
    const double boundary_front = body_front + planning_boundary_margin_;
    const double boundary_rear = body_rear + planning_boundary_margin_;
    const double boundary_left = body_left + planning_boundary_margin_;
    const double boundary_right = body_right + planning_boundary_margin_;
    std::vector<geometry_msgs::msg::Point> boundary_local_points{
      makePoint(boundary_front, boundary_left, 0.0),
      makePoint(boundary_front, -boundary_right, 0.0),
      makePoint(-boundary_rear, -boundary_right, 0.0),
      makePoint(-boundary_rear, boundary_left, 0.0),
      makePoint(boundary_front, boundary_left, 0.0)};
    boundary_marker.points = boundary_local_points;
    markers.markers.emplace_back(boundary_marker);

    avg_msgs::msg::AvgPolygonStamped polygon_msg;
    polygon_msg.header.frame_id = map_frame_id_;
    polygon_msg.header.stamp = now;
    for (size_t i = 0; i < 4; ++i) {
      const auto map_point =
        transformLocal(boundary_local_points[i].x, boundary_local_points[i].y, 0.0);
      avg_msgs::msg::AvgPoint32 p32;
      p32.x = map_point.x;
      p32.y = map_point.y;
      p32.z = 0.0;
      polygon_msg.polygon.points.emplace_back(p32);
    }
    boundary_pub_->publish(polygon_msg);

    const auto sensors = getSensorDictionary();
    // HH_260114 Use only axes+label namespaces per sensor to simplify RViz toggles.
    for (const auto & [name, pose] : sensors) {
      const std::string sensor_ns = "sensor/" + name;
      const auto sensor_position = transformLocal(pose.x, pose.y, pose.z);
      const auto sensor_orientation = composeOrientation(pose.roll, pose.pitch, pose.yaw);

      // HH_260507: Keep one namespace row per sensor in RViz by sharing ns for axes+label.
      markers.markers.emplace_back(
        createAxesMarker(
          sensor_ns, marker_id++, sensor_position, sensor_orientation, 0.5, now,
          map_frame_id_));

      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = map_frame_id_;
      text_marker.header.stamp = now;
      text_marker.ns = sensor_ns;
      text_marker.id = marker_id++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = transformLocal(pose.x, pose.y, pose.z + 0.1);
      text_marker.pose.orientation = base_orientation;
      text_marker.scale.z = kLabelScale;
      text_marker.color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
      text_marker.text = name;
      markers.markers.emplace_back(text_marker);
    }

    if (!range_ring_radii_.empty()) {
      const int segments = 96;
      for (const auto radius : range_ring_radii_) {
        if (radius <= 0.0) {
          continue;
        }
        visualization_msgs::msg::Marker ring_marker;
        ring_marker.header.frame_id = map_frame_id_;
        ring_marker.header.stamp = now;
        ring_marker.ns = "range_rings";
        ring_marker.id = marker_id++;
        ring_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ring_marker.action = visualization_msgs::msg::Marker::ADD;
        ring_marker.pose.position = base_translation;
        ring_marker.pose.orientation = base_orientation;
        ring_marker.scale.x = 0.015;
        ring_marker.color = makeColor(0.6f, 0.6f, 0.6f, 0.4f);
        for (int i = 0; i <= segments; ++i) {
          const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
          ring_marker.points.push_back(makePoint(radius * std::cos(theta), radius * std::sin(theta), 0.0));
        }
        markers.markers.emplace_back(std::move(ring_marker));
      }
    }

    marker_pub_->publish(markers);
    publishAvgPlatform(markers, polygon_msg, now);
    last_marker_publish_time_ = now;
  }

  // Creates a PoseStamped message from the current base pose state.
  avg_msgs::msg::AvgPoseStamped makeBasePoseStamped(const rclcpp::Time & stamp) const
  {
    avg_msgs::msg::AvgPoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position.x = base_pose_.x;
    pose.pose.position.y = base_pose_.y;
    pose.pose.position.z = base_pose_.z;
    // HH_260720 - Convert the visualization quaternion into the generated pose explicitly.
    pose.pose.orientation = avg_msgs::conversions::fromRos(
      quaternionFromRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw));
    return pose;
  }

  // Creates a PoseWithCovarianceStamped wrapper around the base pose for status outputs.
  avg_msgs::msg::AvgPoseWithCovarianceStamped makeBasePoseCov(const rclcpp::Time & stamp) const
  {
    avg_msgs::msg::AvgPoseWithCovarianceStamped pose_cov;
    pose_cov.header.stamp = stamp;
    pose_cov.header.frame_id = map_frame_id_;
    pose_cov.pose.pose = makeBasePoseStamped(stamp).pose;
    std::fill(pose_cov.pose.covariance.begin(), pose_cov.pose.covariance.end(), 0.0);
    pose_cov.pose.covariance[0] = 0.25;
    pose_cov.pose.covariance[7] = 0.25;
    pose_cov.pose.covariance[14] = 0.25;
    pose_cov.pose.covariance[21] = 0.1;
    pose_cov.pose.covariance[28] = 0.1;
    pose_cov.pose.covariance[35] = 0.1;
    return pose_cov;
  }

  // Publishes consolidated platform status payload for system-level consumers.
  void publishAvgPlatform(
    const visualization_msgs::msg::MarkerArray & markers,
    const avg_msgs::msg::AvgPolygonStamped & planning_boundary,
    const rclcpp::Time & stamp)
  {
    if (!publish_platform_status_ || !avg_platform_pub_) {
      return;
    }
    avg_msgs::msg::AvgPlatformMsgs msg;
    msg.stamp = stamp;
    msg.state.stamp = stamp;
    msg.state.module_name = "platform";
    msg.state.level = avg_msgs::msg::ModuleState::OK;
    msg.state.message = "robot_visualization";
    msg.robot_markers = avg_msgs::conversions::fromRos(markers);
    msg.planning_boundary = planning_boundary;
    msg.localization_pose = makeBasePoseStamped(stamp);
    msg.localization_pose_cov = makeBasePoseCov(stamp);
    msg.robot_info.robot_specifications.wheelbase = params_.wheelbase;
    msg.robot_info.robot_specifications.track_width = params_.track_width;
    msg.robot_info.robot_specifications.length = params_.length;
    msg.robot_info.robot_specifications.width = params_.width;
    msg.robot_info.robot_specifications.height = params_.height;
    msg.robot_info.robot_specifications.wheel_radius = params_.wheel_radius;
    msg.robot_info.robot_specifications.encoder_resolution = params_.encoder_resolution;
    msg.robot_info.robot_specifications.drive_type = params_.drive_type;
    auto set_sensor_pose = [](const SensorPose & src, auto & dst) {
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.roll = src.roll;
      dst.pitch = src.pitch;
      dst.yaw = src.yaw;
    };
    set_sensor_pose(params_.imu, msg.robot_info.imu);
    set_sensor_pose(params_.gnss, msg.robot_info.gnss);
    set_sensor_pose(params_.lidar, msg.robot_info.lidar);
    // HH_260326: Canonical camera sensor pose.
    set_sensor_pose(params_.camera, msg.robot_info.camera);
    avg_platform_pub_->publish(msg);
  }

  // Exposes known sensor poses as a name->pose map for marker generation.
  std::vector<std::pair<std::string, SensorPose>> getSensorDictionary() const
  {
    // HH_260623 - Expose the same canonical sensor names used by sensor_kit TF frames.
    return {
      {"imu", params_.imu},
      {"gnss", params_.gnss},
      {"lidar", params_.lidar},
      {"camera/front", params_.camera_front},
      {"camera/rear", params_.camera_rear},
      {"radar/front1", params_.radar_front1},
      {"radar/front2", params_.radar_front2},
      {"radar/left1", params_.radar_left1},
      {"radar/left2", params_.radar_left2},
      {"radar/right1", params_.radar_right1},
      {"radar/right2", params_.radar_right2},
      {"radar/rear", params_.radar_rear}
    };
  }

  // Updates base pose from `/localization/initialpose` and republishes markers immediately.
  void onInitialPose(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const auto & pose = msg->pose.pose;
    base_pose_.x = pose.position.x;
    base_pose_.y = pose.position.y;
    const double pose_z = pose_z_source_ == "pose" ? pose.position.z : 0.0;
    base_pose_.z = pose_z + ground_z_offset_ + mapGroundOffset();
    tf2::Quaternion q;
    // HH_260720 - Convert the generated orientation only at the tf2 ROS API boundary.
    tf2::fromMsg(avg_msgs::conversions::toRos(pose.orientation), q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    base_pose_.roll = roll;
    base_pose_.pitch = pitch;
    // HH_260409: Apply configurable platform heading offset (e.g., -90deg clockwise).
    base_pose_.yaw = normalizeAngle(yaw + heading_yaw_offset_rad_);
    publishBaseTransform();
    publishMarkers();
    // HH_260304-00:00 Suppress per-fix initial-pose spam while GNSS pose is streaming.
  }

  // HH_260720 - Convert the RViz initial-pose input once before internal processing.
  void onInitialPoseRos(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    onInitialPose(std::make_shared<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      avg_msgs::conversions::fromRos(*msg)));
  }

  // Adapts localization pose topic into initialpose-format handling and freshness tracking.
  void onLocalizationPose(const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr msg)
  {
    auto converted = std::make_shared<avg_msgs::msg::AvgPoseWithCovarianceStamped>();
    converted->header = msg->header;
    converted->pose.pose = msg->pose;
    // HH_260408: Preserve latest localization yaw for GNSS fallback path.
    // GNSS pose often has identity orientation, so keep IMU/localization heading.
    last_localization_orientation_ = msg->pose.orientation;
    has_localization_orientation_ = true;
    onInitialPose(converted);
    last_localization_pose_stamp_ = this->get_clock()->now();
    has_localization_pose_ = true;
  }

  // Uses GNSS pose only as fallback when localization is stale or unavailable.
  void onGnssPose(const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr msg)
  {
    // HH_260327: GNSS is fallback-only input for platform marker alignment.
    // If localization pose is fresh, keep marker anchored to localization.
    if (pose_source_mode_ != "localization_with_gnss_fallback") {
      return;
    }
    if (has_localization_pose_) {
      const auto age_sec = (this->get_clock()->now() - last_localization_pose_stamp_).seconds();
      if (age_sec <= localization_pose_timeout_s_) {
        return;
      }
    }
    auto converted = std::make_shared<avg_msgs::msg::AvgPoseWithCovarianceStamped>();
    converted->header = msg->header;
    converted->pose.pose = msg->pose;
    // HH_260408: Keep heading stable while using GNSS position fallback.
    // Without this, marker yaw can snap back to identity orientation.
    if (has_localization_orientation_) {
      converted->pose.pose.orientation = last_localization_orientation_;
    }
    onInitialPose(converted);
  }

  // Returns map-derived ground Z offset when available, otherwise zero.
  double mapGroundOffset() const
  {
    return (ground_z_source_ == "lanelet_map" && map_ground_ready_) ? map_ground_z_ : 0.0;
  }

  // Samples lanelet map marker heights once to estimate ground Z for visualization placement.
  void onMapMarkers(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
  {
    if (ground_z_source_ != "lanelet_map" || map_ground_ready_) {
      return;
    }
    constexpr size_t sample_limit = 1000;
    for (const auto & marker : msg->markers) {
      if (marker.ns.rfind("lanelet/", 0) != 0) {
        continue;
      }
      for (const auto & point : marker.points) {
        map_ground_sum_ += point.z;
        ++map_ground_samples_;
        if (map_ground_samples_ >= sample_limit) {
          break;
        }
      }
      if (map_ground_samples_ >= sample_limit) {
        break;
      }
    }
    if (map_ground_samples_ >= sample_limit) {
      map_ground_z_ = map_ground_sum_ / static_cast<double>(map_ground_samples_);
      map_ground_ready_ = true;
      base_pose_.z = map_ground_z_ + ground_z_offset_;
      RCLCPP_INFO(
        get_logger(), "detected lanelet ground height %.3f m", map_ground_z_);
      if (map_marker_sub_) {
        map_marker_sub_.reset();
      }
      publishMarkers(true);
    }
  }

  RobotParams params_;
  PoseRPY base_pose_;
  double planning_boundary_margin_{0.10};  // HH_260623 - Default to measured body_extents planning margin.
  double body_scale_factor_{1.0};
  double ground_z_offset_{0.0};
  std::string ground_z_source_{"lanelet_map"};
  std::string pose_z_source_{"ground"};
  bool publish_tf_{false};
  std::vector<double> range_ring_radii_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string marker_topic_;
  std::string boundary_topic_;
  std::string platform_status_topic_;
  std::string localization_pose_topic_;
  std::string gnss_pose_topic_;
  std::string pose_source_mode_{"localization_with_gnss_fallback"};
  double localization_pose_timeout_s_{1.0};
  double heading_yaw_offset_deg_{0.0};
  double heading_yaw_offset_rad_{0.0};
  bool show_chassis_marker_{true};
  bool group_robot_marker_namespaces_{true};
  bool has_localization_pose_{false};
  rclcpp::Time last_localization_pose_stamp_{0, 0, RCL_ROS_TIME};
  avg_msgs::msg::AvgQuaternion last_localization_orientation_{};
  bool has_localization_orientation_{false};
  double marker_publish_period_s_{1.0};
  rclcpp::Time last_marker_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgPolygonStamped>::SharedPtr boundary_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgPlatformMsgs>::SharedPtr avg_platform_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr localization_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr map_marker_sub_;

  double map_ground_z_{0.0};
  double map_ground_sum_{0.0};
  size_t map_ground_samples_{0};
  bool map_ground_ready_{false};
  bool publish_platform_status_{false};

  // Composes base orientation with local sensor offset orientation.
  geometry_msgs::msg::Quaternion composeOrientation(double roll, double pitch, double yaw) const
  {
    tf2::Quaternion base_q;
    base_q.setRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw);
    tf2::Quaternion offset_q;
    offset_q.setRPY(roll, pitch, yaw);
    tf2::Quaternion q = base_q * offset_q;
    q.normalize();
    return tf2::toMsg(q);
  }

  // Creates an RGB axis marker (X=red, Y=green, Z=blue) anchored at a point/orientation.
  visualization_msgs::msg::Marker createAxesMarker(
    const std::string & ns, int32_t id,
    const geometry_msgs::msg::Point & origin,
    const geometry_msgs::msg::Quaternion & orientation,
    double length, const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.04;

    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    tf2::Matrix3x3 rot(q);
    tf2::Vector3 origin_vec(origin.x, origin.y, origin.z);

    auto tipPoint = [&](const tf2::Vector3 & axis) {
      tf2::Vector3 tip = origin_vec + rot * axis;
      geometry_msgs::msg::Point p;
      p.x = tip.x();
      p.y = tip.y();
      p.z = tip.z();
      return p;
    };

    marker.points.push_back(origin);
    marker.points.push_back(tipPoint(tf2::Vector3(length, 0.0, 0.0)));
    auto color_x = makeColor(1.0f, 0.0f, 0.0f, 0.9f);
    marker.colors.push_back(color_x);
    marker.colors.push_back(color_x);

    marker.points.push_back(origin);
    marker.points.push_back(tipPoint(tf2::Vector3(0.0, length, 0.0)));
    auto color_y = makeColor(0.0f, 1.0f, 0.0f, 0.9f);
    marker.colors.push_back(color_y);
    marker.colors.push_back(color_y);

    marker.points.push_back(origin);
    marker.points.push_back(tipPoint(tf2::Vector3(0.0, 0.0, length)));
    auto color_z = makeColor(0.0f, 0.6f, 1.0f, 0.9f);
    marker.colors.push_back(color_z);
    marker.colors.push_back(color_z);
    return marker;
  }
};

}  // namespace camrod

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::RobotVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
