#ifndef RANGER_BASE_STEERING_TRANSITION_POLICY_HPP_
#define RANGER_BASE_STEERING_TRANSITION_POLICY_HPP_

// HH_260729 / TODOLIST 13 - Keep translational speed bounded while rate-limited
// wheels still point away from the newest command. This prevents old steering
// geometry from carrying the platform across a centerline after the controller
// changes sign.

#include <algorithm>
#include <cmath>

namespace westonrobot
{

inline double SteeringTransitionVelocityScale(
  const double target_angle_rad,
  const double limited_angle_rad,
  const bool enabled,
  const double full_speed_error_rad,
  const double stop_error_rad,
  const double minimum_scale)
{
  if (!enabled) {
    return 1.0;
  }
  const double lower_error = std::max(0.0, full_speed_error_rad);
  const double upper_error = std::max(lower_error + 1.0e-6, stop_error_rad);
  const double bounded_minimum = std::max(0.0, std::min(1.0, minimum_scale));
  const double error = std::abs(target_angle_rad - limited_angle_rad);
  if (error <= lower_error) {
    return 1.0;
  }
  if (error >= upper_error) {
    return bounded_minimum;
  }
  const double ratio = (error - lower_error) / (upper_error - lower_error);
  return 1.0 - ratio * (1.0 - bounded_minimum);
}

// HH_260818 - A velocity floor that is useful for ordinary Ackermann tracking
// is unsafe while changing between longitudinal and parallel wheel geometry.
// Keep those mode changes stationary until the rate-limited command reaches
// the requested wheel angle; the regular transition envelope resumes after it
// settles.
inline double SteeringModeTransitionVelocityScale(
  const double regular_scale,
  const bool mode_transition_active,
  const double target_angle_rad,
  const double limited_angle_rad,
  const double ready_error_rad)
{
  if (!mode_transition_active ||
    std::abs(target_angle_rad - limited_angle_rad) <=
    std::max(0.0, ready_error_rad))
  {
    return std::max(0.0, std::min(1.0, regular_scale));
  }
  return 0.0;
}

}  // namespace westonrobot

#endif  // RANGER_BASE_STEERING_TRANSITION_POLICY_HPP_
