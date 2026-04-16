// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/VelocityFactor.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VELOCITY_FACTOR__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VELOCITY_FACTOR__STRUCT_H_

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
  autoware_adapi_v1_msgs__msg__VelocityFactor__UNKNOWN = 0
};

/// Constant 'APPROACHING'.
/**
  * constants for status
 */
enum
{
  autoware_adapi_v1_msgs__msg__VelocityFactor__APPROACHING = 1
};

/// Constant 'STOPPED'.
enum
{
  autoware_adapi_v1_msgs__msg__VelocityFactor__STOPPED = 2
};

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'behavior'
// Member 'sequence'
// Member 'detail'
#include "rosidl_runtime_c/string.h"
// Member 'cooperation'
#include "autoware_adapi_v1_msgs/msg/detail/cooperation_status__struct.h"

// constants for array fields with an upper bound
// cooperation
enum
{
  autoware_adapi_v1_msgs__msg__VelocityFactor__cooperation__MAX_SIZE = 1
};

/// Struct defined in msg/VelocityFactor in the package autoware_adapi_v1_msgs.
/**
  * constants for common use
 */
typedef struct autoware_adapi_v1_msgs__msg__VelocityFactor
{
  /// variables
  geometry_msgs__msg__Pose pose;
  float distance;
  uint16_t status;
  rosidl_runtime_c__String behavior;
  rosidl_runtime_c__String sequence;
  rosidl_runtime_c__String detail;
  autoware_adapi_v1_msgs__msg__CooperationStatus__Sequence cooperation;
} autoware_adapi_v1_msgs__msg__VelocityFactor;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__VelocityFactor.
typedef struct autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence
{
  autoware_adapi_v1_msgs__msg__VelocityFactor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__VelocityFactor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VELOCITY_FACTOR__STRUCT_H_
