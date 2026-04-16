// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_planning_msgs:srv/SetPreferredPrimitive.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__STRUCT_H_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'preferred_primitives'
#include "autoware_planning_msgs/msg/detail/lanelet_primitive__struct.h"
// Member 'uuid'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in srv/SetPreferredPrimitive in the package autoware_planning_msgs.
typedef struct autoware_planning_msgs__srv__SetPreferredPrimitive_Request
{
  autoware_planning_msgs__msg__LaneletPrimitive__Sequence preferred_primitives;
  /// reset flag for preferred primitives in route
  /// If set to true, this signals to mission_planner that the preferred-primitives have been reverted to those of the original path
  bool reset;
  /// ID of the route that will be modified.
  unique_identifier_msgs__msg__UUID uuid;
} autoware_planning_msgs__srv__SetPreferredPrimitive_Request;

// Struct for a sequence of autoware_planning_msgs__srv__SetPreferredPrimitive_Request.
typedef struct autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence
{
  autoware_planning_msgs__srv__SetPreferredPrimitive_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_planning_msgs__srv__SetPreferredPrimitive_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "autoware_common_msgs/msg/detail/response_status__struct.h"

/// Struct defined in srv/SetPreferredPrimitive in the package autoware_planning_msgs.
typedef struct autoware_planning_msgs__srv__SetPreferredPrimitive_Response
{
  autoware_common_msgs__msg__ResponseStatus status;
} autoware_planning_msgs__srv__SetPreferredPrimitive_Response;

// Struct for a sequence of autoware_planning_msgs__srv__SetPreferredPrimitive_Response.
typedef struct autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence
{
  autoware_planning_msgs__srv__SetPreferredPrimitive_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_planning_msgs__srv__SetPreferredPrimitive_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__STRUCT_H_
