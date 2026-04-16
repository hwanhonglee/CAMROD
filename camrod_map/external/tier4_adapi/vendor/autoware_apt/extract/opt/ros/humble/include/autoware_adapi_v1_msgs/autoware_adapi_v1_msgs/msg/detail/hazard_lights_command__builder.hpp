// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/HazardLightsCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HAZARD_LIGHTS_COMMAND__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HAZARD_LIGHTS_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/hazard_lights_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_HazardLightsCommand_command
{
public:
  explicit Init_HazardLightsCommand_command(::autoware_adapi_v1_msgs::msg::HazardLightsCommand & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::HazardLightsCommand command(::autoware_adapi_v1_msgs::msg::HazardLightsCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::HazardLightsCommand msg_;
};

class Init_HazardLightsCommand_stamp
{
public:
  Init_HazardLightsCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HazardLightsCommand_command stamp(::autoware_adapi_v1_msgs::msg::HazardLightsCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_HazardLightsCommand_command(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::HazardLightsCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::HazardLightsCommand>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_HazardLightsCommand_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HAZARD_LIGHTS_COMMAND__BUILDER_HPP_
