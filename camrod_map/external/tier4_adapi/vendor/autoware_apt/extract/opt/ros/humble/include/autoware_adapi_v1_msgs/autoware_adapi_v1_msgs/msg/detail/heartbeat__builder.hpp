// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_Heartbeat_seq
{
public:
  explicit Init_Heartbeat_seq(::autoware_adapi_v1_msgs::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::Heartbeat seq(::autoware_adapi_v1_msgs::msg::Heartbeat::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::Heartbeat msg_;
};

class Init_Heartbeat_stamp
{
public:
  Init_Heartbeat_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Heartbeat_seq stamp(::autoware_adapi_v1_msgs::msg::Heartbeat::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Heartbeat_seq(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::Heartbeat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::Heartbeat>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_Heartbeat_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_
