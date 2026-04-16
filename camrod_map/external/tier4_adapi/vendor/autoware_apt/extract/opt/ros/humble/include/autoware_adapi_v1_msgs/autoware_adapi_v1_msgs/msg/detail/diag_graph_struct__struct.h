// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__STRUCT_H_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__STRUCT_H_

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
// Member 'id'
#include "rosidl_runtime_c/string.h"
// Member 'nodes'
#include "autoware_adapi_v1_msgs/msg/detail/diag_node_struct__struct.h"
// Member 'diags'
#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_struct__struct.h"
// Member 'links'
#include "autoware_adapi_v1_msgs/msg/detail/diag_link_struct__struct.h"

/// Struct defined in msg/DiagGraphStruct in the package autoware_adapi_v1_msgs.
typedef struct autoware_adapi_v1_msgs__msg__DiagGraphStruct
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String id;
  autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence nodes;
  autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence diags;
  autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence links;
} autoware_adapi_v1_msgs__msg__DiagGraphStruct;

// Struct for a sequence of autoware_adapi_v1_msgs__msg__DiagGraphStruct.
typedef struct autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence
{
  autoware_adapi_v1_msgs__msg__DiagGraphStruct * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autoware_adapi_v1_msgs__msg__DiagGraphStruct__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_GRAPH_STRUCT__STRUCT_H_
