// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_planning_msgs:srv/SetLaneletRoute.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_LANELET_ROUTE__TRAITS_HPP_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_LANELET_ROUTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_planning_msgs/srv/detail/set_lanelet_route__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'segments'
#include "autoware_planning_msgs/msg/detail/lanelet_segment__traits.hpp"
// Member 'uuid'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace autoware_planning_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLaneletRoute_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: goal_pose
  {
    out << "goal_pose: ";
    to_flow_style_yaml(msg.goal_pose, out);
    out << ", ";
  }

  // member: segments
  {
    if (msg.segments.size() == 0) {
      out << "segments: []";
    } else {
      out << "segments: [";
      size_t pending_items = msg.segments.size();
      for (auto item : msg.segments) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: uuid
  {
    out << "uuid: ";
    to_flow_style_yaml(msg.uuid, out);
    out << ", ";
  }

  // member: allow_modification
  {
    out << "allow_modification: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_modification, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLaneletRoute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: goal_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_pose:\n";
    to_block_style_yaml(msg.goal_pose, out, indentation + 2);
  }

  // member: segments
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.segments.size() == 0) {
      out << "segments: []\n";
    } else {
      out << "segments:\n";
      for (auto item : msg.segments) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: uuid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uuid:\n";
    to_block_style_yaml(msg.uuid, out, indentation + 2);
  }

  // member: allow_modification
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "allow_modification: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_modification, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLaneletRoute_Request & msg, bool use_flow_style = false)
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

}  // namespace autoware_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use autoware_planning_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_planning_msgs::srv::SetLaneletRoute_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_planning_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_planning_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const autoware_planning_msgs::srv::SetLaneletRoute_Request & msg)
{
  return autoware_planning_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_planning_msgs::srv::SetLaneletRoute_Request>()
{
  return "autoware_planning_msgs::srv::SetLaneletRoute_Request";
}

template<>
inline const char * name<autoware_planning_msgs::srv::SetLaneletRoute_Request>()
{
  return "autoware_planning_msgs/srv/SetLaneletRoute_Request";
}

template<>
struct has_fixed_size<autoware_planning_msgs::srv::SetLaneletRoute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_planning_msgs::srv::SetLaneletRoute_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_planning_msgs::srv::SetLaneletRoute_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'status'
#include "autoware_common_msgs/msg/detail/response_status__traits.hpp"

namespace autoware_planning_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLaneletRoute_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    to_flow_style_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLaneletRoute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status:\n";
    to_block_style_yaml(msg.status, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLaneletRoute_Response & msg, bool use_flow_style = false)
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

}  // namespace autoware_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use autoware_planning_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_planning_msgs::srv::SetLaneletRoute_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_planning_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_planning_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const autoware_planning_msgs::srv::SetLaneletRoute_Response & msg)
{
  return autoware_planning_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_planning_msgs::srv::SetLaneletRoute_Response>()
{
  return "autoware_planning_msgs::srv::SetLaneletRoute_Response";
}

template<>
inline const char * name<autoware_planning_msgs::srv::SetLaneletRoute_Response>()
{
  return "autoware_planning_msgs/srv/SetLaneletRoute_Response";
}

template<>
struct has_fixed_size<autoware_planning_msgs::srv::SetLaneletRoute_Response>
  : std::integral_constant<bool, has_fixed_size<autoware_common_msgs::msg::ResponseStatus>::value> {};

template<>
struct has_bounded_size<autoware_planning_msgs::srv::SetLaneletRoute_Response>
  : std::integral_constant<bool, has_bounded_size<autoware_common_msgs::msg::ResponseStatus>::value> {};

template<>
struct is_message<autoware_planning_msgs::srv::SetLaneletRoute_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autoware_planning_msgs::srv::SetLaneletRoute>()
{
  return "autoware_planning_msgs::srv::SetLaneletRoute";
}

template<>
inline const char * name<autoware_planning_msgs::srv::SetLaneletRoute>()
{
  return "autoware_planning_msgs/srv/SetLaneletRoute";
}

template<>
struct has_fixed_size<autoware_planning_msgs::srv::SetLaneletRoute>
  : std::integral_constant<
    bool,
    has_fixed_size<autoware_planning_msgs::srv::SetLaneletRoute_Request>::value &&
    has_fixed_size<autoware_planning_msgs::srv::SetLaneletRoute_Response>::value
  >
{
};

template<>
struct has_bounded_size<autoware_planning_msgs::srv::SetLaneletRoute>
  : std::integral_constant<
    bool,
    has_bounded_size<autoware_planning_msgs::srv::SetLaneletRoute_Request>::value &&
    has_bounded_size<autoware_planning_msgs::srv::SetLaneletRoute_Response>::value
  >
{
};

template<>
struct is_service<autoware_planning_msgs::srv::SetLaneletRoute>
  : std::true_type
{
};

template<>
struct is_service_request<autoware_planning_msgs::srv::SetLaneletRoute_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autoware_planning_msgs::srv::SetLaneletRoute_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_LANELET_ROUTE__TRAITS_HPP_
