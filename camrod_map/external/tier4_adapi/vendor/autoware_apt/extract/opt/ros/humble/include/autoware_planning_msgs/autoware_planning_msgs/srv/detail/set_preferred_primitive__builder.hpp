// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_planning_msgs:srv/SetPreferredPrimitive.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__BUILDER_HPP_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_planning_msgs/srv/detail/set_preferred_primitive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPreferredPrimitive_Request_uuid
{
public:
  explicit Init_SetPreferredPrimitive_Request_uuid(::autoware_planning_msgs::srv::SetPreferredPrimitive_Request & msg)
  : msg_(msg)
  {}
  ::autoware_planning_msgs::srv::SetPreferredPrimitive_Request uuid(::autoware_planning_msgs::srv::SetPreferredPrimitive_Request::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetPreferredPrimitive_Request msg_;
};

class Init_SetPreferredPrimitive_Request_reset
{
public:
  explicit Init_SetPreferredPrimitive_Request_reset(::autoware_planning_msgs::srv::SetPreferredPrimitive_Request & msg)
  : msg_(msg)
  {}
  Init_SetPreferredPrimitive_Request_uuid reset(::autoware_planning_msgs::srv::SetPreferredPrimitive_Request::_reset_type arg)
  {
    msg_.reset = std::move(arg);
    return Init_SetPreferredPrimitive_Request_uuid(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetPreferredPrimitive_Request msg_;
};

class Init_SetPreferredPrimitive_Request_preferred_primitives
{
public:
  Init_SetPreferredPrimitive_Request_preferred_primitives()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPreferredPrimitive_Request_reset preferred_primitives(::autoware_planning_msgs::srv::SetPreferredPrimitive_Request::_preferred_primitives_type arg)
  {
    msg_.preferred_primitives = std::move(arg);
    return Init_SetPreferredPrimitive_Request_reset(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetPreferredPrimitive_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::srv::SetPreferredPrimitive_Request>()
{
  return autoware_planning_msgs::srv::builder::Init_SetPreferredPrimitive_Request_preferred_primitives();
}

}  // namespace autoware_planning_msgs


namespace autoware_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPreferredPrimitive_Response_status
{
public:
  Init_SetPreferredPrimitive_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_planning_msgs::srv::SetPreferredPrimitive_Response status(::autoware_planning_msgs::srv::SetPreferredPrimitive_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetPreferredPrimitive_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::srv::SetPreferredPrimitive_Response>()
{
  return autoware_planning_msgs::srv::builder::Init_SetPreferredPrimitive_Response_status();
}

}  // namespace autoware_planning_msgs

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__BUILDER_HPP_
