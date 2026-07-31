/**
* @file ranger_messenger.hpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

//std and c++ inlclude
#include <string>
#include <memory>
#include <cmath>
#include <vector>

//ros include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

//third libaray inclue
#include "ugv_sdk/details/robot_base/ranger_base.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include <eigen3/Eigen/Core>

//user msg include
#include <ranger_msgs/msg/system_state.hpp>
#include <ranger_msgs/msg/motion_state.hpp>
#include <ranger_msgs/msg/actuator_state_array.hpp>

#include "ranger_msgs/msg/actuator_state.hpp"
#include "ranger_msgs/msg/driver_state.hpp"
#include "ranger_msgs/msg/motor_state.hpp"

#include "ranger_base/ranger_params.hpp"

namespace westonrobot {
class RangerROSMessenger : public std::enable_shared_from_this<RangerROSMessenger>
{
  struct RobotParams {
    double track;
    double wheelbase;
    double max_linear_speed;
    double max_angular_speed;
    double max_speed_cmd;
    double max_steer_angle_central;
    double max_steer_angle_parallel;
    double max_steer_angle_ackermann;
    double max_round_angle;
    double min_turn_radius;
  };

  enum class RangerSubType { kRanger = 0, kRangerMiniV1, kRangerMiniV2 ,kRangerMiniV3};

 public:
  RangerROSMessenger(rclcpp::Node::SharedPtr& node);

  void Run();

 private:
  void LoadParameters();
  void SetupSubscription();
  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);
  void TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  double LimitSteeringAngle(double target_angle);
  rcl_interfaces::msg::SetParametersResult OnParametersChanged(
    const std::vector<rclcpp::Parameter>& parameters);
  double CalculateSteeringAngle(geometry_msgs::msg::Twist msg, double& radius);
  void UpdateOdometry(double linear, double angular, double angle, double dt);
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_;
  RobotParams robot_params_;

  // constants
  const double steer_angle_tolerance_ = 0.005;  // ~+-0.287 degrees

  // parameters
  std::string robot_model_;
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  int update_rate_;
  bool publish_odom_tf_;
  // HH_260727 - Maximum rate used while the wheel direction changes between longitudinal
  // and lateral motion. This is a dynamic ROS parameter.
  double steering_transition_rate_radps_;
  // HH_260729 - Do not command full translation while the rate-limited wheel
  // angle still represents an older controller command.
  bool steering_transition_velocity_scale_enabled_;
  double steering_transition_full_speed_error_rad_;
  double steering_transition_stop_error_rad_;
  double steering_transition_min_velocity_scale_;
  // HH_260731 - Never publish zero covariance for wheel velocity. A zero
  // covariance is treated as near-perfect certainty by robot_localization.
  double odom_linear_velocity_stddev_mps_;
  double odom_angular_velocity_stddev_radps_;

  // Feedback mode only. Command selection uses a callback-local value so a
  // requested transition cannot make CAN feedback use the wrong kinematics.
  uint8_t motion_mode_ = 0;
  bool parking_mode_;
  bool steering_command_initialized_ = false;
  double last_steering_command_rad_ = 0.0;
  rclcpp::Time last_steering_command_time_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  rclcpp::Publisher<ranger_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::MotionState>::SharedPtr motion_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::ActuatorStateArray>::SharedPtr actuator_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // odom variables
  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
