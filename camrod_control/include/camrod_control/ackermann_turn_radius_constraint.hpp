#pragma once

#include <cmath>

#include "avg_msgs/msg/avg_twist.hpp"

namespace camrod_control {

struct AckermannTurnRadiusConstraintResult {
  avg_msgs::msg::AvgTwist command;
  bool valid{true};
  bool constrained{false};
  double original_radius_m{0.0};
};

// Keep a longitudinal arc inside the downstream Ranger Ackermann envelope.
//
// This belongs before the final collision projection: the safety gate must
// evaluate the same curvature that the physical adapter executes. Pure yaw
// remains available for ZERO_TURN and lateral commands remain available for
// CRAB. A non-positive minimum disables this simulator-only constraint so the
// production command contract is unchanged.
inline AckermannTurnRadiusConstraintResult constrainAckermannTurnRadius(
    const avg_msgs::msg::AvgTwist &input,
    const double minimum_turn_radius_m,
    const double lateral_deadband_mps = 0.02,
    const double angular_epsilon_radps = 1.0e-6,
    const double minimum_translation_mps = 0.02,
    const double maximum_radius_correction_m = 0.02) {
  AckermannTurnRadiusConstraintResult result;
  result.command = input;

  // A disabled production profile is an exact identity, including unusual
  // values that continue to be handled by the existing downstream contract.
  if (minimum_turn_radius_m == 0.0) {
    return result;
  }
  if (!std::isfinite(minimum_turn_radius_m) ||
      minimum_turn_radius_m < 0.0 ||
      !std::isfinite(lateral_deadband_mps) || lateral_deadband_mps < 0.0 ||
      !std::isfinite(angular_epsilon_radps) || angular_epsilon_radps < 0.0 ||
      !std::isfinite(minimum_translation_mps) ||
      minimum_translation_mps < 0.0 ||
      !std::isfinite(maximum_radius_correction_m) ||
      maximum_radius_correction_m < 0.0 || !std::isfinite(input.linear.x) ||
      !std::isfinite(input.linear.y) || !std::isfinite(input.angular.z)) {
    result.valid = false;
    return result;
  }

  const double linear_x = std::abs(input.linear.x);
  const double angular_z = std::abs(input.angular.z);
  // Match the adapter's strict lateral and angular comparisons.  Keep pure
  // yaw and sub-translation commands unchanged so explicit ZERO_TURN remains
  // available and no near-zero Ackermann command is manufactured.
  if (linear_x <= minimum_translation_mps ||
      angular_z < angular_epsilon_radps ||
      std::abs(input.linear.y) > lateral_deadband_mps) {
    return result;
  }

  const double maximum_angular_z = linear_x / minimum_turn_radius_m;
  result.original_radius_m = linear_x / angular_z;
  if (angular_z <= maximum_angular_z) {
    return result;
  }
  // This is a mode-boundary stabilizer, not a general curvature controller.
  // Far tighter arcs remain untouched and therefore retain the adapter's
  // explicit ZERO_TURN behavior.
  if (minimum_turn_radius_m - result.original_radius_m >
      maximum_radius_correction_m) {
    return result;
  }

  // Nudge one representable value toward zero.  The downstream adapter uses
  // a strict radius comparison; this prevents a round-trip division from
  // reconstructing a value infinitesimally below the accepted boundary.
  result.command.angular.z = std::copysign(
      std::nextafter(maximum_angular_z, 0.0), input.angular.z);
  result.constrained = true;
  return result;
}

} // namespace camrod_control
