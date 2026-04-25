#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace camrod
{

// Pose container used by each sensor mount relative to `sensor_kit_base_link`.
// Angles are stored in radians inside runtime structures.
struct SensorPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double roll{0.0};    // rad
  double pitch{0.0};   // rad
  double yaw{0.0};     // rad
};

// Full robot geometry and sensor mount specification used across packages.
struct RobotParams
{
  // --- Robot Geometry ---
  double wheelbase{1.10};
  double track_width{0.65};
  double length{1.40};
  double width{0.70};
  double height{1.20};

  double wheel_radius{0.15};
  int encoder_resolution{2048};

  std::string drive_type{"ackermann"};

  // --- Sensors ---
  SensorPose imu;
  SensorPose gnss;
  SensorPose lidar;
  // HH_260326: Canonical camera pose.
  SensorPose camera;
};

// Loads declared ROS parameters into a `RobotParams` snapshot.
RobotParams loadRobotParams(rclcpp::Node * node);

} // namespace camrod
