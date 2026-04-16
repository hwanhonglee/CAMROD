// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_planning_msgs:srv/ClearRoute.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__CLEAR_ROUTE__BUILDER_HPP_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__CLEAR_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_planning_msgs/srv/detail/clear_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_planning_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::srv::ClearRoute_Request>()
{
  return ::autoware_planning_msgs::srv::ClearRoute_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autoware_planning_msgs


namespace autoware_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_ClearRoute_Response_status
{
public:
  Init_ClearRoute_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_planning_msgs::srv::ClearRoute_Response status(::autoware_planning_msgs::srv::ClearRoute_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_planning_msgs::srv::ClearRoute_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::srv::ClearRoute_Response>()
{
  return autoware_planning_msgs::srv::builder::Init_ClearRoute_Response_status();
}

}  // namespace autoware_planning_msgs

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__CLEAR_ROUTE__BUILDER_HPP_
