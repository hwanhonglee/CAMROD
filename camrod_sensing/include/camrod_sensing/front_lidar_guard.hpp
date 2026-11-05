#pragma once

#include <cmath>
#include <limits>

namespace camrod::sensing
{

// HH_261102 - Measured body coordinates, not the LiDAR origin or a class detector, bound the near-field guard.
// HJ_261105 - The guard is an explicit robot_center_link box. Defaults match
// the previous body-front + 0.30 m, body-width, ground + 0.05 m..body-top box.
// x_min and z_min are exclusive so the body face and ground band stay outside.
struct FrontLidarGuardConfig
{
  double x_min_m{0.70837};
  double x_max_m{1.00837};
  double y_min_m{-0.53495};
  double y_max_m{0.53505};
  double z_min_m{-0.10275};
  double z_max_m{0.94188};
};

inline bool validFrontLidarGuardConfig(const FrontLidarGuardConfig & config)
{
  return std::isfinite(config.x_min_m) && std::isfinite(config.x_max_m) &&
         std::isfinite(config.y_min_m) && std::isfinite(config.y_max_m) &&
         std::isfinite(config.z_min_m) && std::isfinite(config.z_max_m) &&
         config.x_min_m > 0.0 && config.x_max_m > config.x_min_m &&
         config.y_max_m > config.y_min_m && config.z_max_m > config.z_min_m;
}

inline bool isFrontLidarGuardPoint(
  const double x, const double y, const double z, const FrontLidarGuardConfig & config)
{
  const double epsilon = std::numeric_limits<float>::epsilon();
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         x > config.x_min_m + epsilon && x <= config.x_max_m + epsilon &&
         y >= config.y_min_m - epsilon && y <= config.y_max_m + epsilon &&
         z > config.z_min_m + epsilon && z <= config.z_max_m + epsilon;
}

}  // namespace camrod::sensing
