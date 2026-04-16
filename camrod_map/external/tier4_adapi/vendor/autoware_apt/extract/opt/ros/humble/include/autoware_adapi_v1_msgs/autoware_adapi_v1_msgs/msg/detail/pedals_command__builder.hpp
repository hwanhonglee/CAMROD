// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/PedalsCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__PEDALS_COMMAND__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__PEDALS_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/pedals_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_PedalsCommand_brake
{
public:
  explicit Init_PedalsCommand_brake(::autoware_adapi_v1_msgs::msg::PedalsCommand & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::PedalsCommand brake(::autoware_adapi_v1_msgs::msg::PedalsCommand::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::PedalsCommand msg_;
};

class Init_PedalsCommand_throttle
{
public:
  explicit Init_PedalsCommand_throttle(::autoware_adapi_v1_msgs::msg::PedalsCommand & msg)
  : msg_(msg)
  {}
  Init_PedalsCommand_brake throttle(::autoware_adapi_v1_msgs::msg::PedalsCommand::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_PedalsCommand_brake(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::PedalsCommand msg_;
};

class Init_PedalsCommand_stamp
{
public:
  Init_PedalsCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PedalsCommand_throttle stamp(::autoware_adapi_v1_msgs::msg::PedalsCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_PedalsCommand_throttle(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::PedalsCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::PedalsCommand>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_PedalsCommand_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__PEDALS_COMMAND__BUILDER_HPP_
