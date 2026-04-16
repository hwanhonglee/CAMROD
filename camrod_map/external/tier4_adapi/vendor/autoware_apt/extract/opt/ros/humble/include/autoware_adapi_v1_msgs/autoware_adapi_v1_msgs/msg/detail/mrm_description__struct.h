// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/MrmDescription.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_DESCRIPTION__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_DESCRIPTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MrmDescription in the package autoware_adapi_v1_msgs.
typedef struct autoware_adapi_v1_msgs__msg__MrmDescription
{
  uint16_t behavior;
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String description;
} autoware_adapi_v1_msgs__msg__MrmDescription;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__MrmDescription.
typedef struct autoware_adapi_v1_msgs__msg__MrmDescription__Sequence
{
  autoware_adapi_v1_msgs__msg__MrmDescription * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__MrmDescription__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_DESCRIPTION__STRUCT_H_
