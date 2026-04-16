// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagLinkStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/diag_link_struct__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_DiagLinkStruct_child
{
public:
  explicit Init_DiagLinkStruct_child(::autoware_adapi_v1_msgs::msg::DiagLinkStruct & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::DiagLinkStruct child(::autoware_adapi_v1_msgs::msg::DiagLinkStruct::_child_type arg)
  {
    msg_.child = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLinkStruct msg_;
};

class Init_DiagLinkStruct_parent
{
public:
  Init_DiagLinkStruct_parent()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiagLinkStruct_child parent(::autoware_adapi_v1_msgs::msg::DiagLinkStruct::_parent_type arg)
  {
    msg_.parent = std::move(arg);
    return Init_DiagLinkStruct_child(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagLinkStruct msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::DiagLinkStruct>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_DiagLinkStruct_parent();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__BUILDER_HPP_
