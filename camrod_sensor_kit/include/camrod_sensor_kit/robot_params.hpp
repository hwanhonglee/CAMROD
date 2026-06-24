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
  double track_width{1.07000};
  double length{1.49160};
  double width{1.07000};
  double height{1.09463};

  // HHL_260623 - Measured asymmetric body extents relative to robot_base_link.
  double body_front_extent{1.20137};
  double body_rear_extent{0.29023};
  double body_left_extent{0.53505};
  double body_right_extent{0.53495};
  double body_top_z{0.94188};
  double body_bottom_z{-0.15275};
  double planning_margin{0.10};

  // HHL_260623 - Default wheel radius updated from measured 152.75 mm.
  double wheel_radius{0.15275};
  int encoder_resolution{2048};

  std::string drive_type{"ackermann"};

  // --- Sensors ---
  SensorPose imu;
  SensorPose gnss;
  SensorPose lidar;
  // HH_260326: Canonical camera pose.
  SensorPose camera;
  // HHL_260623 - Canonical dual-camera and seven-radar poses used by sensor_kit TF/RViz.
  SensorPose camera_front;
  SensorPose camera_rear;
  SensorPose radar_front1;
  SensorPose radar_front2;
  SensorPose radar_left1;
  SensorPose radar_left2;
  SensorPose radar_right1;
  SensorPose radar_right2;
  SensorPose radar_rear;
};

// Loads declared ROS parameters into a `RobotParams` snapshot.
RobotParams loadRobotParams(rclcpp::Node * node);

} // namespace camrod
