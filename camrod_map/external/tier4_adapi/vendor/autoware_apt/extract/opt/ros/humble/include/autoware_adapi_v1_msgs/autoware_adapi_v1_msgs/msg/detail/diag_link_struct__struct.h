// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/DiagLinkStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DiagLinkStruct in the package autoware_adapi_v1_msgs.
/**
  * The index of nodes in the graph struct message.
 */
typedef struct autoware_adapi_v1_msgs__msg__DiagLinkStruct
{
  uint32_t parent;
  uint32_t child;
} autoware_adapi_v1_msgs__msg__DiagLinkStruct;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__DiagLinkStruct.
typedef struct autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence
{
  autoware_adapi_v1_msgs__msg__DiagLinkStruct * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__STRUCT_H_
