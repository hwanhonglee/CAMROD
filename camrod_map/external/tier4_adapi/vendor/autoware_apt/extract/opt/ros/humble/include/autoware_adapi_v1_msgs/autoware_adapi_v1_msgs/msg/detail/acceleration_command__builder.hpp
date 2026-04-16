// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/AccelerationCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__ACCELERATION_COMMAND__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__ACCELERATION_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/acceleration_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_AccelerationCommand_acceleration
{
public:
  explicit Init_AccelerationCommand_acceleration(::autoware_adapi_v1_msgs::msg::AccelerationCommand & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::AccelerationCommand acceleration(::autoware_adapi_v1_msgs::msg::AccelerationCommand::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::AccelerationCommand msg_;
};

class Init_AccelerationCommand_stamp
{
public:
  Init_AccelerationCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AccelerationCommand_acceleration stamp(::autoware_adapi_v1_msgs::msg::AccelerationCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_AccelerationCommand_acceleration(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::AccelerationCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::AccelerationCommand>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_AccelerationCommand_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__ACCELERATION_COMMAND__BUILDER_HPP_
