#pragma once

#include <cmath>
#include <limits>

namespace camrod::sensing
{

// HH_261102 - Measured body coordinates, not the LiDAR origin or a class detector, bound the near-field guard.
struct FrontLidarGuardConfig
{
  double body_front_m{0.70837};
  double body_left_m{0.53505};
  double body_right_m{0.53495};
  double body_bottom_m{-0.15275};
  double body_top_m{0.94188};
  double distance_m{0.30};
  double ground_clearance_m{0.05};
};

inline bool isFrontLidarGuardPoint(
  const double x, const double y, const double z, const FrontLidarGuardConfig & config)
{
  const double epsilon = std::numeric_limits<float>::epsilon();
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         x > config.body_front_m + epsilon &&
         x <= config.body_front_m + config.distance_m + epsilon &&
         y >= -config.body_right_m - epsilon && y <= config.body_left_m + epsilon &&
         z > config.body_bottom_m + config.ground_clearance_m + epsilon &&
         z <= config.body_top_m + epsilon;
}

}  // namespace camrod::sensing
