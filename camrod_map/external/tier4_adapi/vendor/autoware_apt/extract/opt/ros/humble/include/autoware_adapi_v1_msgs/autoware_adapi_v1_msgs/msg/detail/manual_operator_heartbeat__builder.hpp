// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/ManualOperatorHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/manual_operator_heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_ManualOperatorHeartbeat_ready
{
public:
  explicit Init_ManualOperatorHeartbeat_ready(::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat ready(::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat msg_;
};

class Init_ManualOperatorHeartbeat_stamp
{
public:
  Init_ManualOperatorHeartbeat_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManualOperatorHeartbeat_ready stamp(::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ManualOperatorHeartbeat_ready(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_ManualOperatorHeartbeat_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__BUILDER_HPP_
