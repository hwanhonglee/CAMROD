// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/ManualControlModeStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_CONTROL_MODE_STATUS__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_CONTROL_MODE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/manual_control_mode_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_ManualControlModeStatus_mode
{
public:
  explicit Init_ManualControlModeStatus_mode(::autoware_adapi_v1_msgs::msg::ManualControlModeStatus & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::ManualControlModeStatus mode(::autoware_adapi_v1_msgs::msg::ManualControlModeStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::ManualControlModeStatus msg_;
};

class Init_ManualControlModeStatus_stamp
{
public:
  Init_ManualControlModeStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManualControlModeStatus_mode stamp(::autoware_adapi_v1_msgs::msg::ManualControlModeStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ManualControlModeStatus_mode(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::ManualControlModeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::ManualControlModeStatus>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_ManualControlModeStatus_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_CONTROL_MODE_STATUS__BUILDER_HPP_
