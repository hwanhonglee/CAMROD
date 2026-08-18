#pragma once

// HH_260818 - Keep docking and reverse-parking final-approach speed behavior
// deterministic and independently testable from ROS and parking hardware.

#include <algorithm>

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

}  // namespace camrod_control
