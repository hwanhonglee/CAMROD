// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagLeafStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_DiagLeafStatus_values
{
public:
  explicit Init_DiagLeafStatus_values(::autoware_adapi_v1_msgs::msg::DiagLeafStatus & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::DiagLeafStatus values(::autoware_adapi_v1_msgs::msg::DiagLeafStatus::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStatus msg_;
};

class Init_DiagLeafStatus_hardware_id
{
public:
  explicit Init_DiagLeafStatus_hardware_id(::autoware_adapi_v1_msgs::msg::DiagLeafStatus & msg)
  : msg_(msg)
  {}
  Init_DiagLeafStatus_values hardware_id(::autoware_adapi_v1_msgs::msg::DiagLeafStatus::_hardware_id_type arg)
  {
    msg_.hardware_id = std::move(arg);
    return Init_DiagLeafStatus_values(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStatus msg_;
};

class Init_DiagLeafStatus_message
{
public:
  explicit Init_DiagLeafStatus_message(::autoware_adapi_v1_msgs::msg::DiagLeafStatus & msg)
  : msg_(msg)
  {}
  Init_DiagLeafStatus_hardware_id message(::autoware_adapi_v1_msgs::msg::DiagLeafStatus::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_DiagLeafStatus_hardware_id(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStatus msg_;
};

class Init_DiagLeafStatus_input_level
{
public:
  explicit Init_DiagLeafStatus_input_level(::autoware_adapi_v1_msgs::msg::DiagLeafStatus & msg)
  : msg_(msg)
  {}
  Init_DiagLeafStatus_message input_level(::autoware_adapi_v1_msgs::msg::DiagLeafStatus::_input_level_type arg)
  {
    msg_.input_level = std::move(arg);
    return Init_DiagLeafStatus_message(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStatus msg_;
};

class Init_DiagLeafStatus_level
{
public:
  Init_DiagLeafStatus_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiagLeafStatus_input_level level(::autoware_adapi_v1_msgs::msg::DiagLeafStatus::_level_type arg)
  {
    msg_.level = std::move(arg);
    return Init_DiagLeafStatus_input_level(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::DiagLeafStatus>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_DiagLeafStatus_level();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__BUILDER_HPP_
