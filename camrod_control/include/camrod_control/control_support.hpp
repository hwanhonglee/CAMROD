#pragma once

// HH_260721 - Share explicit CAMROD/ROS boundary conversions and control diagnostics.

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_control
{

inline double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline double yawFromPose(const avg_msgs::msg::AvgPoseStamped & pose)
{
  const auto & orientation = pose.pose.orientation;
  const double sin_yaw = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

inline std::pair<double, double> relativeXy(
  const avg_msgs::msg::AvgPoseStamped & reference,
  const avg_msgs::msg::AvgPoseStamped & target)
{
  const double yaw = yawFromPose(reference);
  const double dx = target.pose.position.x - reference.pose.position.x;
  const double dy = target.pose.position.y - reference.pose.position.y;
  return {
    std::cos(yaw) * dx + std::sin(yaw) * dy,
    -std::sin(yaw) * dx + std::cos(yaw) * dy};
}

inline avg_msgs::msg::AvgPoseStamped poseFromRos(
  const geometry_msgs::msg::PoseStamped & input)
{
  avg_msgs::msg::AvgPoseStamped output;
  output.header.stamp = input.header.stamp;
  output.header.frame_id = input.header.frame_id;
  output.pose.position.x = input.pose.position.x;
  output.pose.position.y = input.pose.position.y;
  output.pose.position.z = input.pose.position.z;
  output.pose.orientation.x = input.pose.orientation.x;
  output.pose.orientation.y = input.pose.orientation.y;
  output.pose.orientation.z = input.pose.orientation.z;
  output.pose.orientation.w = input.pose.orientation.w;
  return output;
}

inline avg_msgs::msg::AvgTwist twistFromRos(const geometry_msgs::msg::Twist & input)
{
  avg_msgs::msg::AvgTwist output;
  output.linear.x = input.linear.x;
  output.linear.y = input.linear.y;
  output.linear.z = input.linear.z;
  output.angular.x = input.angular.x;
  output.angular.y = input.angular.y;
  output.angular.z = input.angular.z;
  return output;
}

inline geometry_msgs::msg::Twist twistToRos(const avg_msgs::msg::AvgTwist & input)
{
  geometry_msgs::msg::Twist output;
  output.linear.x = input.linear.x;
  output.linear.y = input.linear.y;
  output.linear.z = input.linear.z;
  output.angular.x = input.angular.x;
  output.angular.y = input.angular.y;
  output.angular.z = input.angular.z;
  return output;
}

inline avg_msgs::msg::ModuleState makeModuleState(
  rclcpp::Node & node,
  const std::string & module_name,
  const uint8_t level,
  const std::string & message)
{
  avg_msgs::msg::ModuleState output;
  output.stamp = node.get_clock()->now();
  output.module_name = module_name;
  output.level = level;
  output.message = message;
  return output;
}

inline diagnostic_msgs::msg::DiagnosticArray makeDiagnostics(
  rclcpp::Node & node,
  const std::string & name,
  const std::string & category,
  const uint8_t level,
  const std::string & message,
  const std::vector<std::pair<std::string, std::string>> & values = {},
  const std::string & hardware_id = "camrod_control")
{
  diagnostic_msgs::msg::DiagnosticArray output;
  output.header.stamp = node.get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = hardware_id;
  status.level = level;
  status.message = message;
  status.values.push_back(diagnostic_msgs::msg::KeyValue().set__key("category").set__value(category));
  status.values.push_back(diagnostic_msgs::msg::KeyValue().set__key("module").set__value(category));
  for (const auto & value : values) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = value.first;
    entry.value = value.second;
    status.values.push_back(entry);
  }
  output.status.push_back(status);
  return output;
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion output;
  output.z = std::sin(yaw * 0.5);
  output.w = std::cos(yaw * 0.5);
  return output;
}

}  // namespace camrod_control
