// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_struct__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_DiagGraphStruct_links
{
public:
  explicit Init_DiagGraphStruct_links(::autoware_adapi_v1_msgs::msg::DiagGraphStruct & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::DiagGraphStruct links(::autoware_adapi_v1_msgs::msg::DiagGraphStruct::_links_type arg)
  {
    msg_.links = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStruct msg_;
};

class Init_DiagGraphStruct_diags
{
public:
  explicit Init_DiagGraphStruct_diags(::autoware_adapi_v1_msgs::msg::DiagGraphStruct & msg)
  : msg_(msg)
  {}
  Init_DiagGraphStruct_links diags(::autoware_adapi_v1_msgs::msg::DiagGraphStruct::_diags_type arg)
  {
    msg_.diags = std::move(arg);
    return Init_DiagGraphStruct_links(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStruct msg_;
};

class Init_DiagGraphStruct_nodes
{
public:
  explicit Init_DiagGraphStruct_nodes(::autoware_adapi_v1_msgs::msg::DiagGraphStruct & msg)
  : msg_(msg)
  {}
  Init_DiagGraphStruct_diags nodes(::autoware_adapi_v1_msgs::msg::DiagGraphStruct::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_DiagGraphStruct_diags(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStruct msg_;
};

class Init_DiagGraphStruct_id
{
public:
  explicit Init_DiagGraphStruct_id(::autoware_adapi_v1_msgs::msg::DiagGraphStruct & msg)
  : msg_(msg)
  {}
  Init_DiagGraphStruct_nodes id(::autoware_adapi_v1_msgs::msg::DiagGraphStruct::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DiagGraphStruct_nodes(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStruct msg_;
};

class Init_DiagGraphStruct_stamp
{
public:
  Init_DiagGraphStruct_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiagGraphStruct_id stamp(::autoware_adapi_v1_msgs::msg::DiagGraphStruct::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_DiagGraphStruct_id(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::DiagGraphStruct msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::DiagGraphStruct>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_DiagGraphStruct_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__BUILDER_HPP_
