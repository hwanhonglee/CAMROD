// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/HazardLightsCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HAZARD_LIGHTS_COMMAND__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HAZARD_LIGHTS_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'command'
#include "autoware_adapi_v1_msgs/msg/detail/hazard_lights__struct.h"

/// Struct defined in msg/HazardLightsCommand in the package autoware_adapi_v1_msgs.
typedef struct autoware_adapi_v1_msgs__msg__HazardLightsCommand
{
  builtin_interfaces__msg__Time stamp;
  autoware_adapi_v1_msgs__msg__HazardLights command;
} autoware_adapi_v1_msgs__msg__HazardLightsCommand;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__HazardLightsCommand.
typedef struct autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence
{
  autoware_adapi_v1_msgs__msg__HazardLightsCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__HazardLightsCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__HAZARD_LIGHTS_COMMAND__STRUCT_H_
