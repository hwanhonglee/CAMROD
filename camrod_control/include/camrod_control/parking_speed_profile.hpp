#pragma once

// HH_260818 - Keep docking and reverse-parking final-approach speed behavior
// deterministic and independently testable from ROS and parking hardware.

#include <algorithm>
#include <cmath>

namespace camrod_control
{

inline double parkingApproachSpeed(
  const double remaining_distance_m,
  const double slowdown_start_remaining_distance_m,
  const double cruise_speed_mps,
  const double final_speed_mps)
{
  const double cruise = std::max(0.0, cruise_speed_mps);
  const double final = std::max(0.0, std::min(cruise, final_speed_mps));
  const double window = std::max(0.0, slowdown_start_remaining_distance_m);
  if (window <= 1.0e-9 || remaining_distance_m >= window) {
    return cruise;
  }
  const double ratio = std::max(0.0, remaining_distance_m) / window;
  return final + (cruise - final) * ratio;
}

// HH_260819 - AprilTag docking uses the same camera-frame 3D range shown in
// the operator UI.  Keep the slowdown window explicit instead of mixing this
// value with the robot-center parking-axis distances retained for compatibility.
inline double tagApproachRemainingDistance(
  const double tag_distance_m,
  const double translation_stop_tag_distance_m)
{
  return std::max(0.0, tag_distance_m - translation_stop_tag_distance_m);
}

inline double tagApproachSlowdownWindow(
  const double slowdown_start_tag_distance_m,
  const double translation_stop_tag_distance_m)
{
  return std::max(
    0.0, slowdown_start_tag_distance_m - translation_stop_tag_distance_m);
}

// HH_260819 - Bound approach curvature so Ranger keeps a combined reverse and
// Dual-Ackermann steering command.  A tighter radius would be reclassified by
// the platform driver as SPINNING, which discards the requested translation.
inline double limitApproachAngularSpeedForTurnRadius(
  const double requested_angular_speed_radps,
  const double linear_speed_mps,
  const double minimum_turn_radius_m)
{
  const double radius = std::max(0.0, minimum_turn_radius_m);
  if (radius <= 1.0e-9) {
    return requested_angular_speed_radps;
  }
  const double angular_limit = std::abs(linear_speed_mps) / radius;
  return std::clamp(
    requested_angular_speed_radps, -angular_limit, angular_limit);
}

}  // namespace camrod_control
