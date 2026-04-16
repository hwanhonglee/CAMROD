// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagLeafStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STRUCT__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STRUCT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_struct__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_DiagLeafStruct_name
{
public:
  explicit Init_DiagLeafStruct_name(::autoware_adapi_v1_msgs::msg::DiagLeafStruct & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::DiagLeafStruct name(::autoware_adapi_v1_msgs::msg::DiagLeafStruct::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStruct msg_;
};

class Init_DiagLeafStruct_parent
{
public:
  Init_DiagLeafStruct_parent()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiagLeafStruct_name parent(::autoware_adapi_v1_msgs::msg::DiagLeafStruct::_parent_type arg)
  {
    msg_.parent = std::move(arg);
    return Init_DiagLeafStruct_name(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLeafStruct msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::DiagLeafStruct>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_DiagLeafStruct_parent();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STRUCT__BUILDER_HPP_
