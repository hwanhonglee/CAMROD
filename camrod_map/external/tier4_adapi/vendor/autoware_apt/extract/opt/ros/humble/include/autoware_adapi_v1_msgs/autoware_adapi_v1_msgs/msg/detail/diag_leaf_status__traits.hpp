// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagLeafStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__TRAITS_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'values'
#include "autoware_adapi_v1_msgs/msg/detail/kv_string__traits.hpp"

namespace autoware_adapi_v1_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DiagLeafStatus & msg,
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

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: hardware_id
  {
    out << "hardware_id: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_id, out);
    out << ", ";
  }

  // member: values
  {
    if (msg.values.size() == 0) {
      out << "values: []";
    } else {
      out << "values: [";
      size_t pending_items = msg.values.size();
      for (auto item : msg.values) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DiagLeafStatus & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: hardware_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hardware_id: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_id, out);
    out << "\n";
  }

  // member: values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.values.size() == 0) {
      out << "values: []\n";
    } else {
      out << "values:\n";
      for (auto item : msg.values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DiagLeafStatus & msg, bool use_flow_style = false)
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
  const autoware_adapi_v1_msgs::msg::DiagLeafStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_adapi_v1_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_adapi_v1_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_adapi_v1_msgs::msg::DiagLeafStatus & msg)
{
  return autoware_adapi_v1_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_adapi_v1_msgs::msg::DiagLeafStatus>()
{
  return "autoware_adapi_v1_msgs::msg::DiagLeafStatus";
}

template<>
inline const char * name<autoware_adapi_v1_msgs::msg::DiagLeafStatus>()
{
  return "autoware_adapi_v1_msgs/msg/DiagLeafStatus";
}

template<>
struct has_fixed_size<autoware_adapi_v1_msgs::msg::DiagLeafStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_adapi_v1_msgs::msg::DiagLeafStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_adapi_v1_msgs::msg::DiagLeafStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__TRAITS_HPP_
