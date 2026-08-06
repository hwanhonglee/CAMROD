#pragma once

// HH_260721 - Keep planar motion and heading geometry in one explicitly named header.

#include <algorithm>
#include <cmath>
#include <utility>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

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

// HH_260807 - Convert a map-frame return-anchor vector into a constant-speed
// body-frame translation so crab mode can remove both lateral and axial drift.
inline std::pair<double, double> bodyTranslationTowardTarget(
  const double current_x,
  const double current_y,
  const double current_yaw,
  const double target_x,
  const double target_y,
  const double speed)
{
  const double dx = target_x - current_x;
  const double dy = target_y - current_y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1.0e-9 || speed <= 0.0) {
    return {0.0, 0.0};
  }
  const double body_x = std::cos(current_yaw) * dx + std::sin(current_yaw) * dy;
  const double body_y = -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;
  const double scale = speed / distance;
  return {body_x * scale, body_y * scale};
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion output;
  output.z = std::sin(yaw * 0.5);
  output.w = std::cos(yaw * 0.5);
  return output;
}

}  // namespace camrod_control
