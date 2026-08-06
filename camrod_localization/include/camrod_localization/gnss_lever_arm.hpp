#pragma once

#include <cmath>

namespace camrod_localization
{

struct PlanarPosition
{
  double x{0.0};
  double y{0.0};
};

// HH_260806 - Convert the GNSS antenna solution to robot center in the map frame.
// Antenna offsets use ROS body axes: +X forward and +Y left.
inline PlanarPosition antennaPositionToRobotCenter(
  double antenna_x,
  double antenna_y,
  double robot_yaw,
  double antenna_offset_x,
  double antenna_offset_y)
{
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double map_offset_x =
    cos_yaw * antenna_offset_x - sin_yaw * antenna_offset_y;
  const double map_offset_y =
    sin_yaw * antenna_offset_x + cos_yaw * antenna_offset_y;

  return {
    antenna_x - map_offset_x,
    antenna_y - map_offset_y,
  };
}

}  // namespace camrod_localization
