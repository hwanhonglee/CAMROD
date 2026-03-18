#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace camping_cart
{

// each sensor pose & structure
struct SensorPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double roll{0.0};    // rad
  double pitch{0.0};   // rad
  double yaw{0.0};     // rad
};

// Robot Overall Specifications Structure
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
  SensorPose camera_front;
};

// Robot Parameter Loading Function (YAML → RobotParams)
RobotParams loadRobotParams(rclcpp::Node * node);

} // namespace camping_cart
