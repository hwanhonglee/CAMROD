// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_adapi_v1_msgs:msg/MrmRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__TRAITS_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_adapi_v1_msgs/msg/detail/mrm_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autoware_adapi_v1_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MrmRequest & msg,
  std::ostream & out)
{
  out << "{";
  // member: sender
  {
    out << "sender: ";
    rosidl_generator_traits::value_to_yaml(msg.sender, out);
    out << ", ";
  }

  // member: strategy
  {
    out << "strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.strategy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MrmRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sender
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sender: ";
    rosidl_generator_traits::value_to_yaml(msg.sender, out);
    out << "\n";
  }

  // member: strategy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.strategy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MrmRequest & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

namespace rosidl_generator_traits
{

[[deprecated("use autoware_adapi_v1_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_adapi_v1_msgs::msg::MrmRequest & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_adapi_v1_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_adapi_v1_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_adapi_v1_msgs::msg::MrmRequest & msg)
{
  return autoware_adapi_v1_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_adapi_v1_msgs::msg::MrmRequest>()
{
  return "autoware_adapi_v1_msgs::msg::MrmRequest";
}

template<>
inline const char * name<autoware_adapi_v1_msgs::msg::MrmRequest>()
{
  return "autoware_adapi_v1_msgs/msg/MrmRequest";
}

template<>
struct has_fixed_size<autoware_adapi_v1_msgs::msg::MrmRequest>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_adapi_v1_msgs::msg::MrmRequest>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_adapi_v1_msgs::msg::MrmRequest>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__TRAITS_HPP_
