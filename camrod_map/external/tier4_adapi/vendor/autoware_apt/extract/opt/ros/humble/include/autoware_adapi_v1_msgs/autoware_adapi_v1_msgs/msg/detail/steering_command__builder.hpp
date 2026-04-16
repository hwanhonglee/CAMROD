// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/SteeringCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__STEERING_COMMAND__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__STEERING_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/steering_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_SteeringCommand_steering_tire_velocity
{
public:
  explicit Init_SteeringCommand_steering_tire_velocity(::autoware_adapi_v1_msgs::msg::SteeringCommand & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::SteeringCommand steering_tire_velocity(::autoware_adapi_v1_msgs::msg::SteeringCommand::_steering_tire_velocity_type arg)
  {
    msg_.steering_tire_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::SteeringCommand msg_;
};

class Init_SteeringCommand_steering_tire_angle
{
public:
  explicit Init_SteeringCommand_steering_tire_angle(::autoware_adapi_v1_msgs::msg::SteeringCommand & msg)
  : msg_(msg)
  {}
  Init_SteeringCommand_steering_tire_velocity steering_tire_angle(::autoware_adapi_v1_msgs::msg::SteeringCommand::_steering_tire_angle_type arg)
  {
    msg_.steering_tire_angle = std::move(arg);
    return Init_SteeringCommand_steering_tire_velocity(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::SteeringCommand msg_;
};

class Init_SteeringCommand_stamp
{
public:
  Init_SteeringCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SteeringCommand_steering_tire_angle stamp(::autoware_adapi_v1_msgs::msg::SteeringCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_SteeringCommand_steering_tire_angle(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::SteeringCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::SteeringCommand>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_SteeringCommand_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__STEERING_COMMAND__BUILDER_HPP_
