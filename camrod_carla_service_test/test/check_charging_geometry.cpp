// HH_261103 - Execute native body-boundary and cargo policy, not a hand-drawn clearance claim.
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "camrod_sensor_kit/robot_boundary.hpp"
#include "camrod_control/cargo_access_target.hpp"

int main()
{
  const double yaw = 91.7873 * std::acos(-1.0) / 180.0;
  const double sx = -8.47366, sy = 40.391, separation = 0.60;
  const double cx = sx - separation * std::cos(yaw);
  const double cy = sy - separation * std::sin(yaw);
  const std::vector<std::pair<double, double>> zone{
    {-9.61239, 38.2276}, {-10.0387, 42.2318}, {-7.36381, 42.5716}, {-6.88622, 38.5571}};
  camrod::RobotBoundaryShape shape;
  shape.extents = {0.70837, 0.68323, 0.53505, 0.53495};
  const auto boundary = camrod::makeExpandedRobotBoundary(shape, {0.10, 0.10, 0.10, 0.10});
  assert(boundary.size() == 30);
  double radius = 0.0;
  for (const auto & p : boundary) {
    // HH_261103 - Native cargo receives float32 polygon points, including the calibrated planning margin.
    radius = std::max(radius, std::hypot(static_cast<double>(static_cast<float>(p.x)),
                                       static_cast<double>(static_cast<float>(p.y))));
  }
  assert(camrod_control::cargoAccessSweepInsideZone(zone, cx, cy, sx, sy, radius));
  assert(!camrod_control::cargoAccessSweepInsideZone(zone, cx, cy - 2.0, sx, sy, radius));
  camrod_control::DropZoneExitTargetPolicyInput input;
  input.vehicle_x_m = cx; input.vehicle_y_m = cy; input.vehicle_yaw_rad = yaw;
  input.vehicle_frame_id = "map"; input.vehicle_header_stamp_available = true;
  const auto selected = camrod_control::makeCargoAccessTargetSelection({}, input, sx, sy, zone, radius);
  assert(selected.valid && std::abs(selected.initial_forward_m - separation) < 1e-12);
  assert(std::abs(selected.initial_lateral_m) < 1e-12);
  input.vehicle_x_m = sx; input.vehicle_y_m = sy;
  assert(!camrod_control::makeCargoAccessTargetSelection({}, input, sx, sy, zone, radius).valid);
  // HH_261103 - Even a cargo stop 0.20 m early must be outside the unchanged inclusive 0.35 m contact radius.
  assert(separation - 0.20 > 0.35);
  std::cout << std::setprecision(17) << "{\"native_cargo_target_valid\":true,\"wire_body_radius_m\":" << radius
            << ",\"planning_boundary_points\":" << boundary.size() << ",\"charging_body_xy_m\":[" << cx << "," << cy
            << "],\"forward_departure_m\":" << selected.initial_forward_m
            << ",\"minimum_charger_separation_at_cargo_stop_m\":" << separation - .20 << "}\n";
}
