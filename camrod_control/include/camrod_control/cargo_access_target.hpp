#pragma once

// HH_261102 - Cargo access ends at the authored station, never at an inferred road or fixed-distance point.

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "camrod_control/drop_zone_exit_target_policy.hpp"

namespace camrod_control
{

inline bool isCargoAccessExitSource(const std::string & source)
{
  return source == "http:charging_access:leave:site_departure";
}

inline bool cargoAccessSweepInsideZone(
  const std::vector<std::pair<double, double>> & corners,
  const double start_x, const double start_y,
  const double target_x, const double target_y, const double body_radius)
{
  // HH_261102 - Endpoint disks in a convex inset imply containment of the entire straight swept body.
  if (corners.size() < 3U || !std::isfinite(body_radius) || body_radius <= 0.0 ||
      !std::isfinite(start_x) || !std::isfinite(start_y) ||
      !std::isfinite(target_x) || !std::isfinite(target_y)) {
    return false;
  }
  double twice_area = 0.0;
  for (std::size_t index = 0; index < corners.size(); ++index) {
    const auto & a = corners[index];
    const auto & b = corners[(index + 1U) % corners.size()];
    if (!std::isfinite(a.first) || !std::isfinite(a.second)) {
      return false;
    }
    twice_area += a.first * b.second - b.first * a.second;
  }
  if (!std::isfinite(twice_area) || std::abs(twice_area) < 1.0e-8) {
    return false;
  }
  const double winding = twice_area > 0.0 ? 1.0 : -1.0;
  for (std::size_t index = 0; index < corners.size(); ++index) {
    const auto & a = corners[index];
    const auto & b = corners[(index + 1U) % corners.size()];
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length) || length < 1.0e-6) {
      return false;
    }
    for (const auto & corner : corners) {
      if (winding * (dx * (corner.second - a.second) -
          dy * (corner.first - a.first)) < -1.0e-8) {
        return false;  // Concave/self-crossed areas cannot use this convex sweep proof.
      }
    }
    const double start_clearance = winding *
      (dx * (start_y - a.second) - dy * (start_x - a.first)) / length;
    const double target_clearance = winding *
      (dx * (target_y - a.second) - dy * (target_x - a.first)) / length;
    if (start_clearance < body_radius || target_clearance < body_radius) {
      return false;
    }
  }
  return true;
}

inline DropZoneExitTargetSelection makeCargoAccessTargetSelection(
  DropZoneExitTargetPolicyConfig config, DropZoneExitTargetPolicyInput input,
  const double station_x, const double station_y,
  const std::vector<std::pair<double, double>> & corners,
  const double body_radius)
{
  DropZoneExitTargetSelection rejected;
  if (input.vehicle_frame_id != "map") {
    rejected.detail = "cargo access requires map-frame localization";
    return rejected;
  }
  // HH_261102 - Reuse the strict existing ahead/distance/freshness checks without a fallback target.
  config.require_lanelet_target = true;
  config.allow_fixed_distance_fallback = false;
  config.allow_unstamped_pose_fallback = false;
  input.lanelet_pose_available = true;
  input.lanelet_x_m = station_x;
  input.lanelet_y_m = station_y;
  input.lanelet_yaw_rad = input.vehicle_yaw_rad;
  input.lanelet_frame_id = "map";
  input.lanelet_age_s = input.vehicle_age_s;
  input.lanelet_header_stamp_available = input.vehicle_header_stamp_available;
  input.lanelet_header_stamp_age_s = input.vehicle_header_stamp_age_s;
  auto target = makeDropZoneExitTargetSelection(config, input);
  if (!target.valid) {
    target.detail = "cargo station target rejected: " + target.detail;
    return target;
  }
  if (!cargoAccessSweepInsideZone(corners, input.vehicle_x_m, input.vehicle_y_m,
      station_x, station_y, body_radius)) {
    rejected.detail = "cargo access requires a known body boundary and full swept body inside one convex drop zone";
    return rejected;
  }
  target.source = DropZoneExitTargetSource::kStationPose;
  target.detail = "captured authored drop-zone station; hold initial body yaw";
  return target;
}

}  // namespace camrod_control
