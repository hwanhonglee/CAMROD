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

// HH_260818 - Project a goal pair around its stable map anchor using the
// vehicle's current heading. This lets a restarted, 180-degree-reversed robot
// select the opposite crab side without changing the site's map coordinates.
inline std::pair<double, double> relativeXyAtHeading(
  const avg_msgs::msg::AvgPoseStamped & reference,
  const avg_msgs::msg::AvgPoseStamped & target,
  const double current_yaw)
{
  const double dx = target.pose.position.x - reference.pose.position.x;
  const double dy = target.pose.position.y - reference.pose.position.y;
  return {
    std::cos(current_yaw) * dx + std::sin(current_yaw) * dy,
    -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy};
}

// HH_260818 - A turnaround must rotate away from the lane side used for crab
// entry. Reversing the vehicle heading reverses both values together.
inline double turnaroundDirectionForCrab(const double crab_direction)
{
  if (std::abs(crab_direction) <= 1.0e-9) {
    return 0.0;
  }
  return crab_direction > 0.0 ? -1.0 : 1.0;
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

// HH_260818 - Campsite exit must not shorten its lateral clearance by taking a
// diagonal shortcut. Remove body-lateral error first with a pure +/-90 degree
// parallel command, then remove the remaining longitudinal drift with a pure
// straight command. The Ranger driver holds translation while the wheels move
// between those two geometries.
inline std::pair<double, double> bodyAxisPrioritizedTranslationTowardTarget(
  const double current_x,
  const double current_y,
  const double current_yaw,
  const double target_x,
  const double target_y,
  const double maximum_speed,
  const double proportional_gain,
  const double lateral_tolerance)
{
  if (maximum_speed <= 0.0 || proportional_gain <= 0.0) {
    return {0.0, 0.0};
  }
  const double dx = target_x - current_x;
  const double dy = target_y - current_y;
  const double body_x = std::cos(current_yaw) * dx + std::sin(current_yaw) * dy;
  const double body_y = -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;
  const auto axis_command = [maximum_speed, proportional_gain](const double error) {
      return clamp(
        proportional_gain * error, -maximum_speed, maximum_speed);
    };
  if (std::abs(body_y) > std::max(0.0, lateral_tolerance)) {
    return {0.0, axis_command(body_y)};
  }
  return {axis_command(body_x), 0.0};
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion output;
  output.z = std::sin(yaw * 0.5);
  output.w = std::cos(yaw * 0.5);
  return output;
}

}  // namespace camrod_control
