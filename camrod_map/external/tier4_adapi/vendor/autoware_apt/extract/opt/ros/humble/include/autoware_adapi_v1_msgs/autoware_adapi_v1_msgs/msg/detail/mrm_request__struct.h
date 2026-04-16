// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/MrmRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__STRUCT_H_

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
  autoware_adapi_v1_msgs__msg__MrmRequest__UNKNOWN = 0
};

/// Constant 'CANCEL'.
/**
  * Cancel the MRM request.
 */
enum
{
  autoware_adapi_v1_msgs__msg__MrmRequest__CANCEL = 1
};

/// Constant 'DELEGATE'.
/**
  * Delegate the selection of MRM behavior to Autoware.
 */
enum
{
  autoware_adapi_v1_msgs__msg__MrmRequest__DELEGATE = 2
};

// Include directives for member types
// Member 'sender'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MrmRequest in the package autoware_adapi_v1_msgs.
typedef struct autoware_adapi_v1_msgs__msg__MrmRequest
{
  /// The identifier of the request sender.
  rosidl_runtime_c__String sender;
  uint16_t strategy;
} autoware_adapi_v1_msgs__msg__MrmRequest;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__MrmRequest.
typedef struct autoware_adapi_v1_msgs__msg__MrmRequest__Sequence
{
  autoware_adapi_v1_msgs__msg__MrmRequest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__MrmRequest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__STRUCT_H_
