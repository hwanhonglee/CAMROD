// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:srv/SelectManualControlMode.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__SELECT_MANUAL_CONTROL_MODE__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__SELECT_MANUAL_CONTROL_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/srv/detail/select_manual_control_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace srv
{

namespace builder
{

class Init_SelectManualControlMode_Request_mode
{
public:
  Init_SelectManualControlMode_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Request mode(::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Request>()
{
  return autoware_adapi_v1_msgs::srv::builder::Init_SelectManualControlMode_Request_mode();
}

}  // namespace autoware_adapi_v1_msgs


namespace autoware_adapi_v1_msgs
{

namespace srv
{

namespace builder
{

class Init_SelectManualControlMode_Response_status
{
public:
  Init_SelectManualControlMode_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Response status(::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::srv::SelectManualControlMode_Response>()
{
  return autoware_adapi_v1_msgs::srv::builder::Init_SelectManualControlMode_Response_status();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__SELECT_MANUAL_CONTROL_MODE__BUILDER_HPP_
