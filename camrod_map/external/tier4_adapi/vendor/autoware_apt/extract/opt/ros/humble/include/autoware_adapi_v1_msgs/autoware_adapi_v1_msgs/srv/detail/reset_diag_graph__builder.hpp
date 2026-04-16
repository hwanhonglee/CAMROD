// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:srv/ResetDiagGraph.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__RESET_DIAG_GRAPH__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__RESET_DIAG_GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/srv/detail/reset_diag_graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::srv::ResetDiagGraph_Request>()
{
  return ::autoware_adapi_v1_msgs::srv::ResetDiagGraph_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autoware_adapi_v1_msgs


namespace autoware_adapi_v1_msgs
{

namespace srv
{

namespace builder
{

class Init_ResetDiagGraph_Response_status
{
public:
  Init_ResetDiagGraph_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_adapi_v1_msgs::srv::ResetDiagGraph_Response status(::autoware_adapi_v1_msgs::srv::ResetDiagGraph_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::srv::ResetDiagGraph_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::srv::ResetDiagGraph_Response>()
{
  return autoware_adapi_v1_msgs::srv::builder::Init_ResetDiagGraph_Response_status();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__RESET_DIAG_GRAPH__BUILDER_HPP_
