// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagNodeStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__TRAITS_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_adapi_v1_msgs/msg/detail/diag_node_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autoware_adapi_v1_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DiagNodeStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: level
  {
    out << "level: ";
    rosidl_generator_traits::character_value_to_yaml(msg.level, out);
    out << ", ";
  }

  // member: input_level
  {
    out << "input_level: ";
    rosidl_generator_traits::character_value_to_yaml(msg.input_level, out);
    out << ", ";
  }

  // member: latch_level
  {
    out << "latch_level: ";
    rosidl_generator_traits::character_value_to_yaml(msg.latch_level, out);
    out << ", ";
  }

  // member: is_dependent
  {
    out << "is_dependent: ";
    rosidl_generator_traits::value_to_yaml(msg.is_dependent, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DiagNodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level: ";
    rosidl_generator_traits::character_value_to_yaml(msg.level, out);
    out << "\n";
  }

  // member: input_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_level: ";
    rosidl_generator_traits::character_value_to_yaml(msg.input_level, out);
    out << "\n";
  }

  // member: latch_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latch_level: ";
    rosidl_generator_traits::character_value_to_yaml(msg.latch_level, out);
    out << "\n";
  }

  // member: is_dependent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_dependent: ";
    rosidl_generator_traits::value_to_yaml(msg.is_dependent, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DiagNodeStatus & msg, bool use_flow_style = false)
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
  const autoware_adapi_v1_msgs::msg::DiagNodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_adapi_v1_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_adapi_v1_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_adapi_v1_msgs::msg::DiagNodeStatus & msg)
{
  return autoware_adapi_v1_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_adapi_v1_msgs::msg::DiagNodeStatus>()
{
  return "autoware_adapi_v1_msgs::msg::DiagNodeStatus";
}

template<>
inline const char * name<autoware_adapi_v1_msgs::msg::DiagNodeStatus>()
{
  return "autoware_adapi_v1_msgs/msg/DiagNodeStatus";
}

template<>
struct has_fixed_size<autoware_adapi_v1_msgs::msg::DiagNodeStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autoware_adapi_v1_msgs::msg::DiagNodeStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autoware_adapi_v1_msgs::msg::DiagNodeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__TRAITS_HPP_
