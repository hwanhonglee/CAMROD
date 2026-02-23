#include "camping_cart_sensor_kit/robot_params.hpp"  // HH_260109 renamed package
#include <cmath>

namespace camping_cart
{

// Degree to Radians Conversion
static double deg2rad(double d) { return d * M_PI / 180.0; }

RobotParams loadRobotParams(rclcpp::Node * node)
{
  RobotParams params;

  // Robot geometry
  params.wheelbase = node->declare_parameter<double>("robot.wheelbase", 1.10);
  params.track_width = node->declare_parameter<double>("robot.track_width", 0.65);
  params.length = node->declare_parameter<double>("robot.length", 1.40);
  params.width = node->declare_parameter<double>("robot.width", 0.70);
  params.height = node->declare_parameter<double>("robot.height", 1.20);

  params.wheel_radius = node->declare_parameter<double>("robot.wheel_radius", 0.15);
  params.encoder_resolution = node->declare_parameter<int>("robot.encoder_resolution", 2048);
  params.drive_type = node->declare_parameter<std::string>("robot.drive_type", "ackermann");

  // Helper lambda for sensor pose loading
  auto load_pose = [&](const std::string & prefix, SensorPose & p)
  {
    p.x = node->declare_parameter<double>(prefix + ".x", 0.0);
    p.y = node->declare_parameter<double>(prefix + ".y", 0.0);
    p.z = node->declare_parameter<double>(prefix + ".z", 0.0);

    // Input is in degrees, internal storage is in radians
    double roll_deg  = node->declare_parameter<double>(prefix + ".roll", 0.0);
    double pitch_deg = node->declare_parameter<double>(prefix + ".pitch", 0.0);
    double yaw_deg   = node->declare_parameter<double>(prefix + ".yaw", 0.0);

    p.roll  = deg2rad(roll_deg);
    p.pitch = deg2rad(pitch_deg);
    p.yaw   = deg2rad(yaw_deg);
  };

  load_pose("imu", params.imu);
  load_pose("gnss", params.gnss);
  // HH_260220: Keep the sensor kit model limited to lidar + camera_front.
  load_pose("lidar", params.lidar);
  load_pose("camera_front", params.camera_front);

  return params;
}

} // namespace camping_cart
