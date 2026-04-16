// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_planning_msgs:msg/RouteState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__MSG__DETAIL__ROUTE_STATE__BUILDER_HPP_
#define AUTOWARE_PLANNING_MSGS__MSG__DETAIL__ROUTE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_planning_msgs/msg/detail/route_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_RouteState_state
{
public:
  explicit Init_RouteState_state(::autoware_planning_msgs::msg::RouteState & msg)
  : msg_(msg)
  {}
  ::autoware_planning_msgs::msg::RouteState state(::autoware_planning_msgs::msg::RouteState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_planning_msgs::msg::RouteState msg_;
};

class Init_RouteState_stamp
{
public:
  Init_RouteState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RouteState_state stamp(::autoware_planning_msgs::msg::RouteState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RouteState_state(msg_);
  }

private:
  ::autoware_planning_msgs::msg::RouteState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::msg::RouteState>()
{
  return autoware_planning_msgs::msg::builder::Init_RouteState_stamp();
}

}  // namespace autoware_planning_msgs

#endif  // AUTOWARE_PLANNING_MSGS__MSG__DETAIL__ROUTE_STATE__BUILDER_HPP_
