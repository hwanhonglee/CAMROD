// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_planning_msgs:msg/RouteState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__MSG__DETAIL__ROUTE_STATE__STRUCT_H_
#define AUTOWARE_PLANNING_MSGS__MSG__DETAIL__ROUTE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNKNOWN'.
enum
{
  autoware_planning_msgs__msg__RouteState__UNKNOWN = 0
};

/// Constant 'INITIALIZING'.
enum
{
  autoware_planning_msgs__msg__RouteState__INITIALIZING = 1
};

/// Constant 'UNSET'.
enum
{
  autoware_planning_msgs__msg__RouteState__UNSET = 2
};

/// Constant 'ROUTING'.
enum
{
  autoware_planning_msgs__msg__RouteState__ROUTING = 3
};

/// Constant 'SET'.
enum
{
  autoware_planning_msgs__msg__RouteState__SET = 4
};

/// Constant 'REROUTING'.
enum
{
  autoware_planning_msgs__msg__RouteState__REROUTING = 5
};

/// Constant 'ARRIVED'.
enum
{
  autoware_planning_msgs__msg__RouteState__ARRIVED = 6
};

/// Constant 'ABORTED'.
enum
{
  autoware_planning_msgs__msg__RouteState__ABORTED = 7
};

/// Constant 'INTERRUPTED'.
enum
{
  autoware_planning_msgs__msg__RouteState__INTERRUPTED = 8
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/RouteState in the package autoware_planning_msgs.
typedef struct autoware_planning_msgs__msg__RouteState
{
  builtin_interfaces__msg__Time stamp;
  uint8_t state;
} autoware_planning_msgs__msg__RouteState;

// Struct for a sequence of autoware_planning_msgs__msg__RouteState.
typedef struct autoware_planning_msgs__msg__RouteState__Sequence
{
  autoware_planning_msgs__msg__RouteState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_planning_msgs__msg__RouteState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_PLANNING_MSGS__MSG__DETAIL__ROUTE_STATE__STRUCT_H_
