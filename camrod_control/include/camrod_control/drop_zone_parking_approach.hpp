#pragma once

// HH_260904 - Keep the exact lanelet parking-point correction deterministic
// and independently testable from ROS callbacks and platform I/O.

#include <algorithm>
#include <cmath>

namespace camrod_control
{

struct DropZoneParkingApproachConfig
{
  double position_tolerance_m{0.05};
  double proportional_gain{0.8};
  double minimum_speed_mps{0.06};
  double maximum_speed_mps{0.20};
  double maximum_correction_m{0.75};
};

struct DropZoneParkingApproachCommand
{
  bool valid{false};
  bool reached{false};
  double distance_m{0.0};
  double body_forward_error_m{0.0};
  double body_lateral_error_m{0.0};
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
};

inline bool dropZoneParkingApproachConfigIsValid(
  const DropZoneParkingApproachConfig & config)
{
  return std::isfinite(config.position_tolerance_m) &&
         config.position_tolerance_m > 0.0 &&
         std::isfinite(config.proportional_gain) &&
         config.proportional_gain > 0.0 &&
         std::isfinite(config.minimum_speed_mps) &&
         config.minimum_speed_mps >= 0.0 &&
         std::isfinite(config.maximum_speed_mps) &&
         config.maximum_speed_mps >= config.minimum_speed_mps &&
         config.maximum_speed_mps > 0.0 &&
         std::isfinite(config.maximum_correction_m) &&
         config.maximum_correction_m > config.position_tolerance_m;
}

inline DropZoneParkingApproachCommand makeDropZoneParkingApproachCommand(
  const DropZoneParkingApproachConfig & config,
  const double vehicle_x_m,
  const double vehicle_y_m,
  const double vehicle_yaw_rad,
  const double target_x_m,
  const double target_y_m)
{
  DropZoneParkingApproachCommand output;
  if (!dropZoneParkingApproachConfigIsValid(config) ||
    !std::isfinite(vehicle_x_m) || !std::isfinite(vehicle_y_m) ||
    !std::isfinite(vehicle_yaw_rad) || !std::isfinite(target_x_m) ||
    !std::isfinite(target_y_m))
  {
    return output;
  }

  const double world_x_error_m = target_x_m - vehicle_x_m;
  const double world_y_error_m = target_y_m - vehicle_y_m;
  output.distance_m = std::hypot(world_x_error_m, world_y_error_m);
  if (!std::isfinite(output.distance_m) ||
    output.distance_m > config.maximum_correction_m)
  {
    return output;
  }

  const double cosine = std::cos(vehicle_yaw_rad);
  const double sine = std::sin(vehicle_yaw_rad);
  output.body_forward_error_m =
    cosine * world_x_error_m + sine * world_y_error_m;
  output.body_lateral_error_m =
    -sine * world_x_error_m + cosine * world_y_error_m;
  output.valid = true;
  if (output.distance_m <= config.position_tolerance_m) {
    output.reached = true;
    return output;
  }

  const double speed_mps = std::clamp(
    config.proportional_gain * output.distance_m,
    config.minimum_speed_mps,
    config.maximum_speed_mps);
  const double scale = speed_mps / output.distance_m;
  output.linear_x_mps = output.body_forward_error_m * scale;
  output.linear_y_mps = output.body_lateral_error_m * scale;
  return output;
}

}  // namespace camrod_control
