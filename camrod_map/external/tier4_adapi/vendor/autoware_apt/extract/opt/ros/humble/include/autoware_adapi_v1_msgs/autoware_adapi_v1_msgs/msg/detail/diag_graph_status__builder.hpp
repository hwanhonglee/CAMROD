// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STATUS__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_DiagGraphStatus_diags
{
public:
  explicit Init_DiagGraphStatus_diags(::autoware_adapi_v1_msgs::msg::DiagGraphStatus & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::DiagGraphStatus diags(::autoware_adapi_v1_msgs::msg::DiagGraphStatus::_diags_type arg)
  {
    msg_.diags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStatus msg_;
};

class Init_DiagGraphStatus_nodes
{
public:
  explicit Init_DiagGraphStatus_nodes(::autoware_adapi_v1_msgs::msg::DiagGraphStatus & msg)
  : msg_(msg)
  {}
  Init_DiagGraphStatus_diags nodes(::autoware_adapi_v1_msgs::msg::DiagGraphStatus::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_DiagGraphStatus_diags(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStatus msg_;
};

class Init_DiagGraphStatus_id
{
public:
  explicit Init_DiagGraphStatus_id(::autoware_adapi_v1_msgs::msg::DiagGraphStatus & msg)
  : msg_(msg)
  {}
  Init_DiagGraphStatus_nodes id(::autoware_adapi_v1_msgs::msg::DiagGraphStatus::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DiagGraphStatus_nodes(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStatus msg_;
};

class Init_DiagGraphStatus_stamp
{
public:
  Init_DiagGraphStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiagGraphStatus_id stamp(::autoware_adapi_v1_msgs::msg::DiagGraphStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_DiagGraphStatus_id(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::DiagGraphStatus>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_DiagGraphStatus_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STATUS__BUILDER_HPP_
