// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/RtiState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__RTI_STATE__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__RTI_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/rti_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_RtiState_message
{
public:
  explicit Init_RtiState_message(::autoware_adapi_v1_msgs::msg::RtiState & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::RtiState message(::autoware_adapi_v1_msgs::msg::RtiState::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::RtiState msg_;
};

class Init_RtiState_request
{
public:
  explicit Init_RtiState_request(::autoware_adapi_v1_msgs::msg::RtiState & msg)
  : msg_(msg)
  {}
  Init_RtiState_message request(::autoware_adapi_v1_msgs::msg::RtiState::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RtiState_message(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::RtiState msg_;
};

class Init_RtiState_stamp
{
public:
  Init_RtiState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RtiState_request stamp(::autoware_adapi_v1_msgs::msg::RtiState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RtiState_request(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::RtiState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::RtiState>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_RtiState_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__RTI_STATE__BUILDER_HPP_
