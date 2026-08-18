/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

#include <algorithm>

#include "ranger_base/kinematics_model.hpp"
#include "ranger_base/parallel_motion_policy.hpp"
#include "ranger_base/steering_transition_policy.hpp"

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {
// namespace {
// double DegreeToRadian(double x) { return x * M_PI / 180.0; }
// }  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node){

  node_ = node;
  LoadParameters();
  parameter_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(
      &RangerROSMessenger::OnParametersChanged, this, std::placeholders::_1));

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
  } else if (robot_type_ == RangerSubType::kRangerMiniV2) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
  } else if (robot_type_ == RangerSubType::kRangerMiniV3) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
  } else {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(),"Failed to connect to the CAN port");
      return;
    }
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(),"Invalid port name: %s", port_name_.c_str());
    return;
  }

  SetupSubscription();
}

void RangerROSMessenger::Run() {
  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    PublishStateToROS();
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  //load parameter from launch files
  port_name_ = node_->declare_parameter<std::string>("port_name","can0");
  robot_model_ = node_->declare_parameter<std::string>("robot_model","ranger");
  odom_frame_ =  node_->declare_parameter<std::string>("odom_frame","odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf",false);
  steering_transition_rate_radps_ = node_->declare_parameter<double>(
    "steering_transition_rate_radps", 0.5);
  steering_transition_rate_radps_ =
    std::max(0.05, std::min(2.0, steering_transition_rate_radps_));
  steering_transition_velocity_scale_enabled_ = node_->declare_parameter<bool>(
    "steering_transition_velocity_scale_enabled", true);
  steering_transition_full_speed_error_rad_ = node_->declare_parameter<double>(
    "steering_transition_full_speed_error_rad", 0.05);
  steering_transition_stop_error_rad_ = node_->declare_parameter<double>(
    "steering_transition_stop_error_rad", 0.35);
  steering_transition_min_velocity_scale_ = node_->declare_parameter<double>(
    "steering_transition_min_velocity_scale", 0.0);
  steering_mode_transition_stationary_enabled_ = node_->declare_parameter<bool>(
    "steering_mode_transition_stationary_enabled", true);
  steering_mode_transition_ready_error_rad_ = node_->declare_parameter<double>(
    "steering_mode_transition_ready_error_rad", 0.05);
  odom_linear_velocity_stddev_mps_ = node_->declare_parameter<double>(
    "odom_linear_velocity_stddev_mps", 0.05);
  odom_angular_velocity_stddev_radps_ = node_->declare_parameter<double>(
    "odom_angular_velocity_stddev_radps", 0.10);
  steering_transition_full_speed_error_rad_ =
    std::max(0.0, steering_transition_full_speed_error_rad_);
  steering_transition_stop_error_rad_ = std::max(
    steering_transition_full_speed_error_rad_ + 1.0e-6,
    steering_transition_stop_error_rad_);
  steering_transition_min_velocity_scale_ = std::max(
    0.0, std::min(1.0, steering_transition_min_velocity_scale_));
  steering_mode_transition_ready_error_rad_ = std::max(
    0.0, std::min(0.35, steering_mode_transition_ready_error_rad_));
  odom_linear_velocity_stddev_mps_ =
    std::max(1.0e-3, odom_linear_velocity_stddev_mps_);
  odom_angular_velocity_stddev_radps_ =
    std::max(1.0e-3, odom_angular_velocity_stddev_radps_);

  RCLCPP_INFO(node_->get_logger(),
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n steering_transition_rate_radps: %.2f\n "
      "steering_transition_velocity_scale: %d full=%.2f stop=%.2f min=%.2f\n "
      "steering_mode_transition_stationary: %d ready_error=%.2f\n "
      "odom_velocity_stddev: linear=%.3f angular=%.3f\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_, steering_transition_rate_radps_,
      steering_transition_velocity_scale_enabled_,
      steering_transition_full_speed_error_rad_,
      steering_transition_stop_error_rad_,
      steering_transition_min_velocity_scale_,
      steering_mode_transition_stationary_enabled_,
      steering_mode_transition_ready_error_rad_,
      odom_linear_velocity_stddev_mps_,
      odom_angular_velocity_stddev_radps_);

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV1Params::max_steer_angle_ackermann;
  } else {
    if (robot_model_ == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV2Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV2Params::max_steer_angle_ackermann;
    }
    if (robot_model_ == "ranger_mini_v3") {
      robot_type_ = RangerSubType::kRangerMiniV3;

      robot_params_.track = RangerMiniV3Params::track;
      robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV3Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV3Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV3Params::max_steer_angle_ackermann;
    }
     else {
      robot_type_ = RangerSubType::kRanger;

      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerParams::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerParams::max_steer_angle_ackermann;
    }
  }
    parking_mode_ = false;

}

void RangerROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SystemState>("/system_state", 10);
  motion_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::MotionState>("/motion_state", 10);
  actuator_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::ActuatorStateArray>("/actuator_state", 10);
  // HH_260801 / TODOLIST 13 - Publish each controller-to-wheel transition as
  // structured, timestamped data. Rosout throttling cannot preserve every
  // target sign change or align it reliably with pose and CAN feedback.
  steering_transition_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SteeringTransitionState>(
        "/platform/steering_transition_state", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

  // subscriber
  motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&RangerROSMessenger::TwistCmdCallback, this, std::placeholders::_1)
      );
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // HH_260731 - Use the mode reported by the same CAN snapshot for odometry.
  // Previously this assignment happened after UpdateOdometry(), so every mode
  // transition could be interpreted with an earlier/requested kinematic model.
  motion_mode_ = state.motion_mode_state.motion_mode;

  // update odometry
  {
    double dt = (current_time_ - last_time_).seconds();
    UpdateOdometry(state.motion_state.linear_velocity,
                   state.motion_state.angular_velocity,
                   state.motion_state.steering_angle, dt);
    last_time_ = current_time_;
  }

  // publish system state
  {
    ranger_msgs::msg::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    system_state_pub_->publish(system_msg);
  }

  // publish motion mode
  {
    ranger_msgs::msg::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    motion_state_pub_->publish(motion_msg);
  }

  // publish actuator state
  {
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "Angle_5:%f Angle_6:%f Angle_7:%f Angle_8:%f",
    //                 actuator_state.motor_angles.angle_5,
    //                 actuator_state.motor_angles.angle_6,
    //                 actuator_state.motor_angles.angle_7,
    //                 actuator_state.motor_angles.angle_8);
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "speed_1:%f speed_2:%f speed_3:%f speed_4:%f",
    //                 actuator_state.motor_speeds.speed_1,
    //                 actuator_state.motor_speeds.speed_2,
    //                 actuator_state.motor_speeds.speed_3,
    //                 actuator_state.motor_speeds.speed_4);

    ranger_msgs::msg::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;
    const double drive_speeds[4] = {
      actuator_state.motor_speeds.speed_1,
      actuator_state.motor_speeds.speed_2,
      actuator_state.motor_speeds.speed_3,
      actuator_state.motor_speeds.speed_4};
    const double steering_angles[4] = {
      actuator_state.motor_angles.angle_5,
      actuator_state.motor_angles.angle_6,
      actuator_state.motor_angles.angle_7,
      actuator_state.motor_angles.angle_8};
    for (int i = 0; i < 8; i++) {
      ranger_msgs::msg::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state[i].driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state[i].driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state[i].motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state[i].driver_state;

      ranger_msgs::msg::MotorState motor_state_msg;
      motor_state_msg.current = actuator_state.actuator_hs_state[i].current;
      motor_state_msg.pulse_count = actuator_state.actuator_hs_state[i].pulse_count;
      motor_state_msg.rpm = actuator_state.actuator_hs_state[i].rpm;
      // HH_260731 - IDs 0..3 carry drive speed and IDs 4..7 carry steering
      // angle. Publishing speed_1/angle_5 eight times hid the wheel that failed
      // to reach its crab angle and made platform diagnostics misleading.
      if (i < 4) {
        motor_state_msg.motor_speeds = drive_speeds[i];
      } else {
        motor_state_msg.motor_angles = steering_angles[i - 4];
      }
      
      ranger_msgs::msg::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_->publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = std::numeric_limits<uint8_t>::quiet_NaN();

    battery_state_pub_->publish(batt_msg);
  }
}

void RangerROSMessenger::UpdateOdometry(double linear, double angular,
                                        double angle, double dt) {
  // update odometry calculations
  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
    DualAckermanModel::control_type u;
    u.v = linear;
    u.phi = ConvertInnerAngleToCentral(angle);

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
        DualAckermanModel(robot_params_.wheelbase, u), x, 0.0, dt, (dt / 10.0));
    //std::cout<<" steer: "<<angle<<" central: "<<u.phi<<std::endl;
    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      u.phi = M_PI / 2.0;
    } else {
      u.phi = angle;
    }
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    // HH_260731 - Preserve non-zero yaw feedback during imperfect crab motion.
    // The old branch forced theta_dot=0 even when CAN reported rotation.
    theta_ = x[2] + angular * dt;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    SpinningModel::state_type x = {position_x_, position_y_, theta_};
    SpinningModel::control_type u;
    u.w = angular;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
        SpinningModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  }

  // update odometry topics
  geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  const double linear_variance =
      odom_linear_velocity_stddev_mps_ * odom_linear_velocity_stddev_mps_;
  const double angular_variance =
      odom_angular_velocity_stddev_radps_ * odom_angular_velocity_stddev_radps_;
  odom_msg.twist.covariance[0] = linear_variance;
  odom_msg.twist.covariance[7] = linear_variance;
  odom_msg.twist.covariance[14] = linear_variance;
  odom_msg.twist.covariance[21] = angular_variance;
  odom_msg.twist.covariance[28] = angular_variance;
  odom_msg.twist.covariance[35] = angular_variance;

  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z =
        2 * linear * std::sin(ConvertInnerAngleToCentral(angle)) /
        robot_params_.wheelbase;
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    double phi = angle;

    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      phi = M_PI / 2.0;
    }
    odom_msg.twist.twist.linear.x = linear * std::cos(phi);
    odom_msg.twist.twist.linear.y = linear * std::sin(phi);

    // HH_260806 - Publish measured CAN yaw rate. The active EKF fuses this
    // wheel-derived rate with IMU yaw rate between GNSS updates.
    // HH_260807 - Wheel/IMU prediction bridges the physical dual-GNSS 5 Hz
    // correction epochs; localization output itself remains 20 Hz.
    odom_msg.twist.twist.angular.z = angular;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = angular;
  }

  odom_pub_->publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
  // HH_260729 - Keep the transition-envelope input defined even when the
  // compiler cannot prove that motion-mode selection initializes this branch.
  double steer_cmd = 0.0;
  double radius;

  // analyze Twist msg and switch motion_mode
  // check for parking mode, only applicable to RangerMiniV2
  if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2) {
    return;
  }

  uint8_t command_mode = MotionState::MOTION_MODE_DUAL_ACKERMAN;
  if (msg->linear.y != 0) {
    if (msg->linear.x == 0.0 && robot_type_ == RangerSubType::kRangerMiniV1) {
      command_mode = MotionState::MOTION_MODE_SIDE_SLIP;
      robot_->SetMotionMode(MotionState::MOTION_MODE_SIDE_SLIP);
    } else {
      command_mode = MotionState::MOTION_MODE_PARALLEL;
      robot_->SetMotionMode(MotionState::MOTION_MODE_PARALLEL);
    }
  } else {
    steer_cmd = CalculateSteeringAngle(*msg, radius);
    // Use minimum turn radius to switch between dual ackerman and spinning mode
    if (radius < robot_params_.min_turn_radius) {
      command_mode = MotionState::MOTION_MODE_SPINNING;
      robot_->SetMotionMode(MotionState::MOTION_MODE_SPINNING);
    } else {
      command_mode = MotionState::MOTION_MODE_DUAL_ACKERMAN;
      robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
    }
  }
  if (command_mode != MotionState::MOTION_MODE_DUAL_ACKERMAN &&
    command_mode != MotionState::MOTION_MODE_PARALLEL)
  {
    // HH_260818 - A campsite zero-turn physically leaves the previous parallel
    // wheel geometry. Re-seed the next longitudinal/parallel limiter from CAN
    // feedback so CRAB_OUT cannot inherit a stale +/-90 degree command and move
    // before the wheels return to the newly requested geometry.
    steering_command_initialized_ = false;
    translational_mode_initialized_ = false;
    steering_mode_transition_active_ = false;
  }
  // send motion command to robot
  switch (command_mode) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN: {
      if (steer_cmd > robot_params_.max_steer_angle_ackermann) {
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann) {
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      }
      const double target_steering_rad = steer_cmd;
      steer_cmd = LimitSteeringAngle(target_steering_rad);
      const double regular_velocity_scale = SteeringTransitionVelocityScale(
        target_steering_rad, steer_cmd,
        steering_transition_velocity_scale_enabled_,
        steering_transition_full_speed_error_rad_,
        steering_transition_stop_error_rad_,
        steering_transition_min_velocity_scale_);
      const double velocity_scale = ApplySteeringModeTransitionVelocityScale(
        command_mode, target_steering_rad, steer_cmd, regular_velocity_scale);
      const double commanded_speed_mps = msg->linear.x * velocity_scale;
      robot_->SetMotionCommand(commanded_speed_mps, steer_cmd);
      PublishSteeringTransitionState(
        *msg, command_mode, true, target_steering_rad, steer_cmd,
        velocity_scale, commanded_speed_mps);
      if (velocity_scale < 0.999) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "steering transition limits velocity: mode=ackermann target=%.3f "
          "limited=%.3f scale=%.2f",
          target_steering_rad, steer_cmd, velocity_scale);
      }
      break;
    }
    case MotionState::MOTION_MODE_PARALLEL: {
      // HH_260731 - atan(y/x) plus last_nonzero_x produced the wrong vector in
      // rear quadrants and made pure crab direction depend on command history.
      const auto parallel = ResolveParallelMotionCommand(
        msg->linear.x, msg->linear.y);
      steer_cmd = parallel.steering_angle_rad;
      // HH_260729 - Resolve the final parallel-steering sign before limiting.
      // Limiting first allowed the later sign flip to bypass the configured
      // wheel-angle slew rate during longitudinal/lateral transitions.
      if (steer_cmd > robot_params_.max_steer_angle_parallel) {
        steer_cmd = robot_params_.max_steer_angle_parallel;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_parallel) {
        steer_cmd = -robot_params_.max_steer_angle_parallel;
      }
      const double target_steering_rad = steer_cmd;
      steer_cmd = LimitSteeringAngle(target_steering_rad);
      const double regular_velocity_scale = SteeringTransitionVelocityScale(
        target_steering_rad, steer_cmd,
        steering_transition_velocity_scale_enabled_,
        steering_transition_full_speed_error_rad_,
        steering_transition_stop_error_rad_,
        steering_transition_min_velocity_scale_);
      const double velocity_scale = ApplySteeringModeTransitionVelocityScale(
        command_mode, target_steering_rad, steer_cmd, regular_velocity_scale);
      const double commanded_speed_mps = parallel.signed_speed * velocity_scale;
      robot_->SetMotionCommand(commanded_speed_mps, steer_cmd);
      PublishSteeringTransitionState(
        *msg, command_mode, true, target_steering_rad, steer_cmd,
        velocity_scale, commanded_speed_mps);
      if (velocity_scale < 0.999) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "steering transition limits velocity: mode=parallel target=%.3f "
          "limited=%.3f scale=%.2f",
          target_steering_rad, steer_cmd, velocity_scale);
      }
      break;
    }
    case MotionState::MOTION_MODE_SPINNING: {
      double a_v = msg->angular.z;
      if (a_v > robot_params_.max_angular_speed) {
        a_v = robot_params_.max_angular_speed;
      }
      if (a_v < -robot_params_.max_angular_speed) {
        a_v = -robot_params_.max_angular_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, a_v);
      PublishSteeringTransitionState(
        *msg, command_mode, false, 0.0, 0.0, 1.0, 0.0);
      break;
    }
    case MotionState::MOTION_MODE_SIDE_SLIP: {
      double l_v = msg->linear.y;
      if (l_v > robot_params_.max_linear_speed) {
        l_v = robot_params_.max_linear_speed;
      }
      if (l_v < -robot_params_.max_linear_speed) {
        l_v = -robot_params_.max_linear_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, l_v);
      PublishSteeringTransitionState(
        *msg, command_mode, false, 0.0, 0.0, 1.0, l_v);
      break;
    }
  }
}

void RangerROSMessenger::PublishSteeringTransitionState(
  const geometry_msgs::msg::Twist & msg,
  const uint8_t command_mode,
  const bool steering_angle_valid,
  const double target_steering_rad,
  const double limited_steering_rad,
  const double translation_scale,
  const double commanded_speed_mps)
{
  ranger_msgs::msg::SteeringTransitionState state;
  state.header.stamp = node_->get_clock()->now();
  state.header.frame_id = base_frame_;
  state.command_mode = command_mode;
  state.steering_angle_valid = steering_angle_valid;
  state.requested_linear_x_mps = msg.linear.x;
  state.requested_linear_y_mps = msg.linear.y;
  state.requested_angular_z_radps = msg.angular.z;
  state.target_steering_rad = target_steering_rad;
  state.limited_steering_rad = limited_steering_rad;
  state.steering_error_rad =
    steering_angle_valid ? target_steering_rad - limited_steering_rad : 0.0;
  state.translation_scale = translation_scale;
  state.commanded_speed_mps = commanded_speed_mps;
  steering_transition_state_pub_->publish(state);
}

double RangerROSMessenger::LimitSteeringAngle(const double target_angle) {
  const auto now = node_->get_clock()->now();
  if (!steering_command_initialized_) {
    // HH_260727 - Start from the measured steering state so restarting the driver while the
    // wheels are lateral cannot create a full-angle command discontinuity.
    last_steering_command_rad_ = robot_->GetRobotState().motion_state.steering_angle;
    last_steering_command_time_ = now;
    steering_command_initialized_ = true;
  }

  double dt = (now - last_steering_command_time_).seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = 1.0 / std::max(1, update_rate_);
  }
  // HH_260727 - A stale cmd_vel stream must not turn into one large steering jump.
  dt = std::min(dt, 0.1);
  const double max_delta = steering_transition_rate_radps_ * dt;
  const double delta = target_angle - last_steering_command_rad_;
  if (delta > max_delta) {
    last_steering_command_rad_ += max_delta;
  } else if (delta < -max_delta) {
    last_steering_command_rad_ -= max_delta;
  } else {
    last_steering_command_rad_ = target_angle;
  }
  last_steering_command_time_ = now;
  return last_steering_command_rad_;
}

double RangerROSMessenger::ApplySteeringModeTransitionVelocityScale(
  const uint8_t command_mode,
  const double target_angle_rad,
  const double limited_angle_rad,
  const double regular_scale)
{
  if (command_mode != MotionState::MOTION_MODE_DUAL_ACKERMAN &&
    command_mode != MotionState::MOTION_MODE_PARALLEL)
  {
    return regular_scale;
  }

  if (!translational_mode_initialized_) {
    // HH_260818 - Do not infer the physical mode from an intermediate limited
    // angle: the first parallel step can already exceed the Ackermann maximum
    // and would incorrectly release the 20% velocity floor. After startup or
    // zero-turn, require the requested angle itself to settle before moving.
    settled_translational_mode_ = command_mode;
    steering_mode_transition_target_ = command_mode;
    steering_mode_transition_active_ =
      std::abs(target_angle_rad - limited_angle_rad) >
      steering_mode_transition_ready_error_rad_;
    translational_mode_initialized_ = true;
  } else if (steering_mode_transition_active_) {
    // A cancel/reversal during steering remains stationary until the newest
    // target is reached; falling back to the ordinary 20% floor here would
    // recreate the diagonal campsite exit.
    steering_mode_transition_target_ = command_mode;
  } else if (command_mode != settled_translational_mode_) {
    steering_mode_transition_target_ = command_mode;
    steering_mode_transition_active_ = true;
  }

  if (!steering_mode_transition_stationary_enabled_) {
    settled_translational_mode_ = command_mode;
    steering_mode_transition_target_ = command_mode;
    steering_mode_transition_active_ = false;
    return regular_scale;
  }

  const double velocity_scale = SteeringModeTransitionVelocityScale(
    regular_scale, steering_mode_transition_active_, target_angle_rad,
    limited_angle_rad, steering_mode_transition_ready_error_rad_);
  if (steering_mode_transition_active_ && velocity_scale > 0.0) {
    settled_translational_mode_ = steering_mode_transition_target_;
    steering_mode_transition_active_ = false;
  }
  return velocity_scale;
}

rcl_interfaces::msg::SetParametersResult RangerROSMessenger::OnParametersChanged(
  const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double requested_rate = steering_transition_rate_radps_;
  bool requested_scaling_enabled = steering_transition_velocity_scale_enabled_;
  double requested_full_speed_error = steering_transition_full_speed_error_rad_;
  double requested_stop_error = steering_transition_stop_error_rad_;
  double requested_minimum_scale = steering_transition_min_velocity_scale_;
  bool requested_stationary_mode_transition =
    steering_mode_transition_stationary_enabled_;
  double requested_mode_transition_ready_error =
    steering_mode_transition_ready_error_rad_;
  bool changed = false;

  for (const auto& parameter : parameters) {
    const auto& name = parameter.get_name();
    if (name == "steering_transition_rate_radps") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "steering_transition_rate_radps must be a double";
        return result;
      }
      requested_rate = parameter.as_double();
      if (!std::isfinite(requested_rate) || requested_rate < 0.05 || requested_rate > 2.0) {
        result.successful = false;
        result.reason = "steering_transition_rate_radps must be in [0.05, 2.0]";
        return result;
      }
      changed = true;
    } else if (name == "steering_transition_velocity_scale_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "steering_transition_velocity_scale_enabled must be a bool";
        return result;
      }
      requested_scaling_enabled = parameter.as_bool();
      changed = true;
    } else if (name == "steering_mode_transition_stationary_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "steering_mode_transition_stationary_enabled must be a bool";
        return result;
      }
      requested_stationary_mode_transition = parameter.as_bool();
      changed = true;
    } else if (name == "steering_mode_transition_ready_error_rad") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "steering_mode_transition_ready_error_rad must be a double";
        return result;
      }
      requested_mode_transition_ready_error = parameter.as_double();
      if (!std::isfinite(requested_mode_transition_ready_error) ||
        requested_mode_transition_ready_error < 0.0 ||
        requested_mode_transition_ready_error > 0.35)
      {
        result.successful = false;
        result.reason =
          "steering_mode_transition_ready_error_rad must be in [0, 0.35]";
        return result;
      }
      changed = true;
    } else if (
      name == "steering_transition_full_speed_error_rad" ||
      name == "steering_transition_stop_error_rad" ||
      name == "steering_transition_min_velocity_scale")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = name + " must be a double";
        return result;
      }
      const double requested = parameter.as_double();
      if (!std::isfinite(requested)) {
        result.successful = false;
        result.reason = name + " must be finite";
        return result;
      }
      if (name == "steering_transition_full_speed_error_rad") {
        requested_full_speed_error = requested;
      } else if (name == "steering_transition_stop_error_rad") {
        requested_stop_error = requested;
      } else {
        requested_minimum_scale = requested;
      }
      changed = true;
    }
  }
  if (requested_full_speed_error < 0.0 ||
    requested_stop_error <= requested_full_speed_error)
  {
    result.successful = false;
    result.reason =
      "steering transition errors require 0 <= full_speed_error < stop_error";
    return result;
  }
  if (requested_minimum_scale < 0.0 || requested_minimum_scale > 1.0) {
    result.successful = false;
    result.reason = "steering_transition_min_velocity_scale must be in [0, 1]";
    return result;
  }
  if (changed) {
    steering_transition_rate_radps_ = requested_rate;
    steering_transition_velocity_scale_enabled_ = requested_scaling_enabled;
    steering_transition_full_speed_error_rad_ = requested_full_speed_error;
    steering_transition_stop_error_rad_ = requested_stop_error;
    steering_transition_min_velocity_scale_ = requested_minimum_scale;
    steering_mode_transition_stationary_enabled_ =
      requested_stationary_mode_transition;
    steering_mode_transition_ready_error_rad_ =
      requested_mode_transition_ready_error;
    RCLCPP_INFO(
      node_->get_logger(),
      "steering transition updated dynamically: rate=%.2f rad/s scale=%s "
      "full=%.2f stop=%.2f min=%.2f mode_stationary=%s ready=%.2f",
      steering_transition_rate_radps_,
      steering_transition_velocity_scale_enabled_ ? "true" : "false",
      steering_transition_full_speed_error_rad_,
      steering_transition_stop_error_rad_,
      steering_transition_min_velocity_scale_,
      steering_mode_transition_stationary_enabled_ ? "true" : "false",
      steering_mode_transition_ready_error_rad_);
  }
  return result;
}


geometry_msgs::msg::Quaternion RangerROSMessenger::createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

double RangerROSMessenger::CalculateSteeringAngle(geometry_msgs::msg::Twist msg,
                                                  double& radius) {
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

  if (angular < 1e-6) {
    radius = std::numeric_limits<double>::infinity(); 
    return 0.0; 
  }
  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1 : -1;

  double l, w, phi_i, x;
  l = robot_params_.wheelbase;
  w = robot_params_.track;
  x = sqrt(radius * radius + (l / 2) * (l / 2));
  // phi_i = atan((l / 2) / (x - w / 2));
  phi_i = atan((l / 2) / radius);

  const double max_phi_rad = 40.0 * M_PI / 180.0;
  phi_i = std::min(phi_i, max_phi_rad);

  return k * phi_i;
}

double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = std::abs(angle);

  phi = std::atan(robot_params_.wheelbase * std::sin(phi_i) /
                  (robot_params_.wheelbase * std::cos(phi_i) +
                   robot_params_.track * std::sin(phi_i)));

  phi *= angle >= 0 ? 1.0 : -1.0;
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = std::abs(angle);
  double phi_i = 0;

  phi_i = std::atan(robot_params_.wheelbase * std::sin(phi) /
                    (robot_params_.wheelbase * std::cos(phi) -
                     robot_params_.track * std::sin(phi)));
  phi_i *= angle >= 0 ? 1.0 : -1.0;
  return phi_i;
}
}  // namespace westonrobot
