#pragma once

// HH_260731 - Convert a requested body-frame XY velocity into Ranger's
// signed-speed + parallel-steering representation without depending on the
// direction of an earlier command.

#include <cmath>

namespace westonrobot
{

struct ParallelMotionCommand
{
  double signed_speed{0.0};
  double steering_angle_rad{0.0};
};

inline ParallelMotionCommand ResolveParallelMotionCommand(
  const double linear_x, const double linear_y)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kHalfPi = kPi * 0.5;

  ParallelMotionCommand output;
  const double speed = std::hypot(linear_x, linear_y);
  if (speed <= 1.0e-9) {
    return output;
  }

  output.signed_speed = speed;
  output.steering_angle_rad = std::atan2(linear_y, linear_x);

  // Ranger parallel steering is limited to +/-90 degrees. A vector in either
  // rear quadrant is represented by reversing wheel speed and rotating the
  // steering angle by 180 degrees into the supported interval.
  if (output.steering_angle_rad > kHalfPi) {
    output.steering_angle_rad -= kPi;
    output.signed_speed = -output.signed_speed;
  } else if (output.steering_angle_rad < -kHalfPi) {
    output.steering_angle_rad += kPi;
    output.signed_speed = -output.signed_speed;
  }
  return output;
}

}  // namespace westonrobot
