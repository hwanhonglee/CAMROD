// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_vehicle_msgs:msg/SteeringReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_VEHICLE_MSGS__MSG__DETAIL__STEERING_REPORT__STRUCT_H_
#define AUTOWARE_VEHICLE_MSGS__MSG__DETAIL__STEERING_REPORT__STRUCT_H_

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

/// Struct defined in msg/SteeringReport in the package autoware_vehicle_msgs.
typedef struct autoware_vehicle_msgs__msg__SteeringReport
{
  builtin_interfaces__msg__Time stamp;
  float steering_tire_angle;
} autoware_vehicle_msgs__msg__SteeringReport;

// Struct for a sequence of autoware_vehicle_msgs__msg__SteeringReport.
typedef struct autoware_vehicle_msgs__msg__SteeringReport__Sequence
{
  autoware_vehicle_msgs__msg__SteeringReport * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_vehicle_msgs__msg__SteeringReport__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_VEHICLE_MSGS__MSG__DETAIL__STEERING_REPORT__STRUCT_H_
