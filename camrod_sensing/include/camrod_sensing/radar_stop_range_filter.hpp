#pragma once

#include <cmath>
#include <limits>

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
  // HH_261102 - Range is float32 on the wire; roundoff must not exclude an exact millimetre cutoff.
  const double tolerance_m =
    std::numeric_limits<float>::epsilon() * std::abs(stop_candidate_max_range_m);
  return
    std::isfinite(range_m) &&
    std::isfinite(sensor_min_range_m) &&
    std::isfinite(sensor_max_range_m) &&
    std::isfinite(stop_candidate_max_range_m) &&
    sensor_min_range_m >= 0.0 &&
    sensor_max_range_m >= sensor_min_range_m &&
    stop_candidate_max_range_m > 0.0 &&
    range_m >= sensor_min_range_m - tolerance_m &&
    range_m <= sensor_max_range_m + tolerance_m &&
    range_m <= stop_candidate_max_range_m + tolerance_m;
}

}  // namespace camrod::sensing::radar_stop_range_filter
