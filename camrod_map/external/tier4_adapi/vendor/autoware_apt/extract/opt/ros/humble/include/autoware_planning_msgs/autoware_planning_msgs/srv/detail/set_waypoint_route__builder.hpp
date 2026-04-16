// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_planning_msgs:srv/SetWaypointRoute.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_WAYPOINT_ROUTE__BUILDER_HPP_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_WAYPOINT_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_planning_msgs/srv/detail/set_waypoint_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_SetWaypointRoute_Request_allow_modification
{
public:
  explicit Init_SetWaypointRoute_Request_allow_modification(::autoware_planning_msgs::srv::SetWaypointRoute_Request & msg)
  : msg_(msg)
  {}
  ::autoware_planning_msgs::srv::SetWaypointRoute_Request allow_modification(::autoware_planning_msgs::srv::SetWaypointRoute_Request::_allow_modification_type arg)
  {
    msg_.allow_modification = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetWaypointRoute_Request msg_;
};

class Init_SetWaypointRoute_Request_uuid
{
public:
  explicit Init_SetWaypointRoute_Request_uuid(::autoware_planning_msgs::srv::SetWaypointRoute_Request & msg)
  : msg_(msg)
  {}
  Init_SetWaypointRoute_Request_allow_modification uuid(::autoware_planning_msgs::srv::SetWaypointRoute_Request::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_SetWaypointRoute_Request_allow_modification(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetWaypointRoute_Request msg_;
};

class Init_SetWaypointRoute_Request_waypoints
{
public:
  explicit Init_SetWaypointRoute_Request_waypoints(::autoware_planning_msgs::srv::SetWaypointRoute_Request & msg)
  : msg_(msg)
  {}
  Init_SetWaypointRoute_Request_uuid waypoints(::autoware_planning_msgs::srv::SetWaypointRoute_Request::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return Init_SetWaypointRoute_Request_uuid(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetWaypointRoute_Request msg_;
};

class Init_SetWaypointRoute_Request_goal_pose
{
public:
  explicit Init_SetWaypointRoute_Request_goal_pose(::autoware_planning_msgs::srv::SetWaypointRoute_Request & msg)
  : msg_(msg)
  {}
  Init_SetWaypointRoute_Request_waypoints goal_pose(::autoware_planning_msgs::srv::SetWaypointRoute_Request::_goal_pose_type arg)
  {
    msg_.goal_pose = std::move(arg);
    return Init_SetWaypointRoute_Request_waypoints(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetWaypointRoute_Request msg_;
};

class Init_SetWaypointRoute_Request_header
{
public:
  Init_SetWaypointRoute_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetWaypointRoute_Request_goal_pose header(::autoware_planning_msgs::srv::SetWaypointRoute_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SetWaypointRoute_Request_goal_pose(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetWaypointRoute_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::srv::SetWaypointRoute_Request>()
{
  return autoware_planning_msgs::srv::builder::Init_SetWaypointRoute_Request_header();
}

}  // namespace autoware_planning_msgs


namespace autoware_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_SetWaypointRoute_Response_status
{
public:
  Init_SetWaypointRoute_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_planning_msgs::srv::SetWaypointRoute_Response status(::autoware_planning_msgs::srv::SetWaypointRoute_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_planning_msgs::srv::SetWaypointRoute_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_planning_msgs::srv::SetWaypointRoute_Response>()
{
  return autoware_planning_msgs::srv::builder::Init_SetWaypointRoute_Response_status();
}

}  // namespace autoware_planning_msgs

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_WAYPOINT_ROUTE__BUILDER_HPP_
