#pragma once

#include <cmath>

namespace camrod::sensing::radar_stop_range_filter
{

// HH_260807 - Keep the sensor observation contract independent from the
// near-proximity stop policy. Boundaries are inclusive to match Range.
inline bool isStopCandidate(
  const double range_m,
  const double sensor_min_range_m,
  const double sensor_max_range_m,
  const double stop_candidate_max_range_m) noexcept
{
  return
    std::isfinite(range_m) &&
    std::isfinite(sensor_min_range_m) &&
    std::isfinite(sensor_max_range_m) &&
    std::isfinite(stop_candidate_max_range_m) &&
    sensor_min_range_m >= 0.0 &&
    sensor_max_range_m >= sensor_min_range_m &&
    stop_candidate_max_range_m > 0.0 &&
    range_m >= sensor_min_range_m &&
    range_m <= sensor_max_range_m &&
    range_m <= stop_candidate_max_range_m;
}

}  // namespace camrod::sensing::radar_stop_range_filter
