#include "camrod_sensor_kit/robot_params.hpp"  // HH_260109 renamed package
#include <cmath>

namespace camrod
{

// Converts degree-based YAML values into radians for runtime math.
static double deg2rad(double d) { return d * M_PI / 180.0; }

// Declares and loads robot geometry/sensor parameters from the current node.
RobotParams loadRobotParams(rclcpp::Node * node)
{
  RobotParams params;

  // Robot geometry
  params.wheelbase = node->declare_parameter<double>("robot.wheelbase", 1.10);
  params.track_width = node->declare_parameter<double>("robot.track_width", params.track_width);
  params.length = node->declare_parameter<double>("robot.length", params.length);
  params.width = node->declare_parameter<double>("robot.width", params.width);
  params.height = node->declare_parameter<double>("robot.height", params.height);
  // HHL_260623 - Load measured asymmetric envelope used by URDF, RViz boundary, and planning safety.
  params.body_front_extent = node->declare_parameter<double>(
    "robot.body_extents.front", params.body_front_extent);
  params.body_rear_extent = node->declare_parameter<double>(
    "robot.body_extents.rear", params.body_rear_extent);
  params.body_left_extent = node->declare_parameter<double>(
    "robot.body_extents.left", params.body_left_extent);
  params.body_right_extent = node->declare_parameter<double>(
    "robot.body_extents.right", params.body_right_extent);
  params.body_top_z = node->declare_parameter<double>(
    "robot.body_extents.top_z", params.body_top_z);
  params.body_bottom_z = node->declare_parameter<double>(
    "robot.body_extents.bottom_z", params.body_bottom_z);
  params.planning_margin = node->declare_parameter<double>(
    "robot.body_extents.planning_margin", params.planning_margin);

  // HHL_260623 - Default wheel radius updated from measured 152.75 mm.
  params.wheel_radius = node->declare_parameter<double>("robot.wheel_radius", 0.15275);
  params.encoder_resolution = node->declare_parameter<int>("robot.encoder_resolution", 2048);
  params.drive_type = node->declare_parameter<std::string>("robot.drive_type", "ackermann");

  // Shared loader for sensor pose groups such as `imu.*`, `gnss.*`, `camera.*`, and `lidar.*`.
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
  auto is_default_pose = [](const SensorPose & p)
  {
    constexpr double eps = 1.0e-9;
    return std::abs(p.x) < eps && std::abs(p.y) < eps && std::abs(p.z) < eps &&
      std::abs(p.roll) < eps && std::abs(p.pitch) < eps && std::abs(p.yaw) < eps;
  };

  load_pose("imu", params.imu);
  load_pose("gnss", params.gnss);
  // HH_260326: Canonical camera pose.
  load_pose("camera", params.camera);
  // HHL_260623 - Load canonical nested camera/radar poses used by sensor_kit.launch.py and xacro.
  load_pose("camera.front", params.camera_front);
  load_pose("camera.rear", params.camera_rear);
  if (is_default_pose(params.camera_front)) {
    params.camera_front = params.camera;
  } else {
    params.camera = params.camera_front;
  }
  load_pose("lidar", params.lidar);
  load_pose("radar.front1", params.radar_front1);
  load_pose("radar.front2", params.radar_front2);
  load_pose("radar.left1", params.radar_left1);
  load_pose("radar.left2", params.radar_left2);
  load_pose("radar.right1", params.radar_right1);
  load_pose("radar.right2", params.radar_right2);
  load_pose("radar.rear", params.radar_rear);

  return params;
}

} // namespace camrod
