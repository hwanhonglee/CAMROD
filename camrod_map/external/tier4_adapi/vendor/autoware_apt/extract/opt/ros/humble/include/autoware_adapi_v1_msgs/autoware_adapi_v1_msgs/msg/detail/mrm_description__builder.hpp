// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/MrmDescription.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_DESCRIPTION__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_DESCRIPTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/mrm_description__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_MrmDescription_description
{
public:
  explicit Init_MrmDescription_description(::autoware_adapi_v1_msgs::msg::MrmDescription & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::MrmDescription description(::autoware_adapi_v1_msgs::msg::MrmDescription::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::MrmDescription msg_;
};

class Init_MrmDescription_name
{
public:
  explicit Init_MrmDescription_name(::autoware_adapi_v1_msgs::msg::MrmDescription & msg)
  : msg_(msg)
  {}
  Init_MrmDescription_description name(::autoware_adapi_v1_msgs::msg::MrmDescription::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MrmDescription_description(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::MrmDescription msg_;
};

class Init_MrmDescription_behavior
{
public:
  Init_MrmDescription_behavior()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MrmDescription_name behavior(::autoware_adapi_v1_msgs::msg::MrmDescription::_behavior_type arg)
  {
    msg_.behavior = std::move(arg);
    return Init_MrmDescription_name(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::MrmDescription msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::MrmDescription>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_MrmDescription_behavior();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_DESCRIPTION__BUILDER_HPP_
