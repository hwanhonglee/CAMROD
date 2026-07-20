#pragma once

// HH_260721 - Keep reverse-axis geometry independent from node runtime state.

#include <cmath>

#include "camrod_control/control_support.hpp"

namespace camrod_control
{

inline double bodyYawForReverseAxis(const double reverse_axis_yaw_rad)
{
  return normalizeAngle(reverse_axis_yaw_rad + M_PI);
}

inline double reverseAxisYawForBody(const double body_yaw_rad)
{
  return normalizeAngle(body_yaw_rad + M_PI);
}

inline double signedDistanceAlongAxis(
  const double origin_x_m,
  const double origin_y_m,
  const double target_x_m,
  const double target_y_m,
  const double axis_yaw_rad)
{
  return
    (target_x_m - origin_x_m) * std::cos(axis_yaw_rad) +
    (target_y_m - origin_y_m) * std::sin(axis_yaw_rad);
}

}  // namespace camrod_control
