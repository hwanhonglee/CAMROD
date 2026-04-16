// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagNodeStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/diag_node_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_DiagNodeStatus_is_dependent
{
public:
  explicit Init_DiagNodeStatus_is_dependent(::autoware_adapi_v1_msgs::msg::DiagNodeStatus & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::DiagNodeStatus is_dependent(::autoware_adapi_v1_msgs::msg::DiagNodeStatus::_is_dependent_type arg)
  {
    msg_.is_dependent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagNodeStatus msg_;
};

class Init_DiagNodeStatus_latch_level
{
public:
  explicit Init_DiagNodeStatus_latch_level(::autoware_adapi_v1_msgs::msg::DiagNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DiagNodeStatus_is_dependent latch_level(::autoware_adapi_v1_msgs::msg::DiagNodeStatus::_latch_level_type arg)
  {
    msg_.latch_level = std::move(arg);
    return Init_DiagNodeStatus_is_dependent(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagNodeStatus msg_;
};

class Init_DiagNodeStatus_input_level
{
public:
  explicit Init_DiagNodeStatus_input_level(::autoware_adapi_v1_msgs::msg::DiagNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DiagNodeStatus_latch_level input_level(::autoware_adapi_v1_msgs::msg::DiagNodeStatus::_input_level_type arg)
  {
    msg_.input_level = std::move(arg);
    return Init_DiagNodeStatus_latch_level(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagNodeStatus msg_;
};

class Init_DiagNodeStatus_level
{
public:
  Init_DiagNodeStatus_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiagNodeStatus_input_level level(::autoware_adapi_v1_msgs::msg::DiagNodeStatus::_level_type arg)
  {
    msg_.level = std::move(arg);
    return Init_DiagNodeStatus_input_level(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagNodeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::DiagNodeStatus>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_DiagNodeStatus_level();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__BUILDER_HPP_
