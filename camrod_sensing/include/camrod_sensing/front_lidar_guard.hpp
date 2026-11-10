#pragma once

#include <cmath>
#include <cstdint>
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

// HJ_261110 - LiDAR noise rejection. A scan counts as an obstacle only when at
// least min_points in-box points are seen, and the guard is confirmed only after
// min_consecutive_scans such scans in a row. Points come after the 3 cm voxel
// filter, so N points are N distinct voxels. The publish timer can see the
// same cloud more than once; a repeated stamp returns the previous decision
// without counting it again. Defaults of 1/1 keep the single-point behavior.
struct FrontLidarGuardNoiseConfig
{
  int min_points{1};
  int min_consecutive_scans{1};
};

inline bool validFrontLidarGuardNoiseConfig(const FrontLidarGuardNoiseConfig & config)
{
  return config.min_points >= 1 && config.min_consecutive_scans >= 1;
}

class FrontLidarGuardNoiseFilter
{
public:
  explicit FrontLidarGuardNoiseFilter(const FrontLidarGuardNoiseConfig & config = {})
  : config_(config) {}

  bool update(const std::int64_t scan_stamp_ns, const int points_in_box)
  {
    if (has_scan_ && scan_stamp_ns == last_scan_stamp_ns_) {
      return confirmed_;
    }
    has_scan_ = true;
    last_scan_stamp_ns_ = scan_stamp_ns;
    if (points_in_box >= config_.min_points) {
      if (consecutive_scans_ < config_.min_consecutive_scans) {
        ++consecutive_scans_;
      }
    } else {
      consecutive_scans_ = 0;
    }
    confirmed_ = consecutive_scans_ >= config_.min_consecutive_scans;
    return confirmed_;
  }

  int consecutiveScans() const {return consecutive_scans_;}

private:
  FrontLidarGuardNoiseConfig config_;
  bool has_scan_{false};
  std::int64_t last_scan_stamp_ns_{0};
  int consecutive_scans_{0};
  bool confirmed_{false};
};

}  // namespace camrod::sensing
