#pragma once

#include <cmath>
#include <limits>

namespace camrod_control {

struct ReverseParkingGoalCheck {
  bool valid{false};
  bool reached{false};
  double xy_error_m{std::numeric_limits<double>::infinity()};
};

// A travel/axis limit is permission to STOP, never evidence that parking
// succeeded. Check the actual station XY disk, including during the bounded
// final approach inside its axial envelope. An outside-disk lateral miss or
// overshoot must not count as arrival.
inline ReverseParkingGoalCheck checkReverseParkingGoal(
    const double vehicle_x, const double vehicle_y,
    const double station_x, const double station_y, const double tolerance_m) {
  ReverseParkingGoalCheck result;
  if (!std::isfinite(vehicle_x) || !std::isfinite(vehicle_y) ||
      !std::isfinite(station_x) || !std::isfinite(station_y) ||
      !std::isfinite(tolerance_m) || tolerance_m <= 0.0) {
    return result;
  }
  result.xy_error_m = std::hypot(station_x - vehicle_x, station_y - vehicle_y);
  result.valid = std::isfinite(result.xy_error_m);
  result.reached = result.valid && result.xy_error_m <= tolerance_m;
  return result;
}

}  // namespace camrod_control
