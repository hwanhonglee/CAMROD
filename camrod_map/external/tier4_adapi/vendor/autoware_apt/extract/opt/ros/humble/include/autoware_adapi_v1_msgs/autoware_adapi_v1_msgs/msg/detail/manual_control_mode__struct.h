// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/ManualControlMode.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_CONTROL_MODE__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_CONTROL_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DISABLED'.
enum
{
  autoware_adapi_v1_msgs__msg__ManualControlMode__DISABLED = 1
};

/// Constant 'PEDALS'.
enum
{
  autoware_adapi_v1_msgs__msg__ManualControlMode__PEDALS = 2
};

/// Constant 'ACCELERATION'.
enum
{
  autoware_adapi_v1_msgs__msg__ManualControlMode__ACCELERATION = 3
};

/// Constant 'VELOCITY'.
enum
{
  autoware_adapi_v1_msgs__msg__ManualControlMode__VELOCITY = 4
};

/// Struct defined in msg/ManualControlMode in the package autoware_adapi_v1_msgs.
typedef struct autoware_adapi_v1_msgs__msg__ManualControlMode
{
  uint8_t mode;
} autoware_adapi_v1_msgs__msg__ManualControlMode;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__ManualControlMode.
typedef struct autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence
{
  autoware_adapi_v1_msgs__msg__ManualControlMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__ManualControlMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MANUAL_CONTROL_MODE__STRUCT_H_
