// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_adapi_v1_msgs:srv/ListMrmDescription.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__LIST_MRM_DESCRIPTION__TRAITS_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__LIST_MRM_DESCRIPTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_adapi_v1_msgs/srv/detail/list_mrm_description__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autoware_adapi_v1_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListMrmDescription_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ListMrmDescription_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ListMrmDescription_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs

namespace rosidl_generator_traits
{

[[deprecated("use autoware_adapi_v1_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_adapi_v1_msgs::srv::ListMrmDescription_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_adapi_v1_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_adapi_v1_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const autoware_adapi_v1_msgs::srv::ListMrmDescription_Request & msg)
{
  return autoware_adapi_v1_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>()
{
  return "autoware_adapi_v1_msgs::srv::ListMrmDescription_Request";
}

template<>
inline const char * name<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>()
{
  return "autoware_adapi_v1_msgs/srv/ListMrmDescription_Request";
}

template<>
struct has_fixed_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'descriptions'
#include "autoware_adapi_v1_msgs/msg/detail/mrm_description__traits.hpp"
// Member 'status'
#include "autoware_adapi_v1_msgs/msg/detail/response_status__traits.hpp"

namespace autoware_adapi_v1_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListMrmDescription_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: descriptions
  {
    if (msg.descriptions.size() == 0) {
      out << "descriptions: []";
    } else {
      out << "descriptions: [";
      size_t pending_items = msg.descriptions.size();
      for (auto item : msg.descriptions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    to_flow_style_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ListMrmDescription_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: descriptions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.descriptions.size() == 0) {
      out << "descriptions: []\n";
    } else {
      out << "descriptions:\n";
      for (auto item : msg.descriptions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status:\n";
    to_block_style_yaml(msg.status, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ListMrmDescription_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs

namespace rosidl_generator_traits
{

[[deprecated("use autoware_adapi_v1_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_adapi_v1_msgs::srv::ListMrmDescription_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_adapi_v1_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_adapi_v1_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const autoware_adapi_v1_msgs::srv::ListMrmDescription_Response & msg)
{
  return autoware_adapi_v1_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>()
{
  return "autoware_adapi_v1_msgs::srv::ListMrmDescription_Response";
}

template<>
inline const char * name<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>()
{
  return "autoware_adapi_v1_msgs/srv/ListMrmDescription_Response";
}

template<>
struct has_fixed_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autoware_adapi_v1_msgs::srv::ListMrmDescription>()
{
  return "autoware_adapi_v1_msgs::srv::ListMrmDescription";
}

template<>
inline const char * name<autoware_adapi_v1_msgs::srv::ListMrmDescription>()
{
  return "autoware_adapi_v1_msgs/srv/ListMrmDescription";
}

template<>
struct has_fixed_size<autoware_adapi_v1_msgs::srv::ListMrmDescription>
  : std::integral_constant<
    bool,
    has_fixed_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>::value &&
    has_fixed_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>::value
  >
{
};

template<>
struct has_bounded_size<autoware_adapi_v1_msgs::srv::ListMrmDescription>
  : std::integral_constant<
    bool,
    has_bounded_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>::value &&
    has_bounded_size<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>::value
  >
{
};

template<>
struct is_service<autoware_adapi_v1_msgs::srv::ListMrmDescription>
  : std::true_type
{
};

template<>
struct is_service_request<autoware_adapi_v1_msgs::srv::ListMrmDescription_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autoware_adapi_v1_msgs::srv::ListMrmDescription_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__LIST_MRM_DESCRIPTION__TRAITS_HPP_
