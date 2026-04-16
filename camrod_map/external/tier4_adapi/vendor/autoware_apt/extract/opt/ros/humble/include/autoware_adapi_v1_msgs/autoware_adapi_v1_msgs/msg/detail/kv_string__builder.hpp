// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/KvString.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__KV_STRING__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__KV_STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/kv_string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_KvString_value
{
public:
  explicit Init_KvString_value(::autoware_adapi_v1_msgs::msg::KvString & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::KvString value(::autoware_adapi_v1_msgs::msg::KvString::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::KvString msg_;
};

class Init_KvString_key
{
public:
  Init_KvString_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KvString_value key(::autoware_adapi_v1_msgs::msg::KvString::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_KvString_value(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::KvString msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::KvString>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_KvString_key();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__KV_STRING__BUILDER_HPP_
