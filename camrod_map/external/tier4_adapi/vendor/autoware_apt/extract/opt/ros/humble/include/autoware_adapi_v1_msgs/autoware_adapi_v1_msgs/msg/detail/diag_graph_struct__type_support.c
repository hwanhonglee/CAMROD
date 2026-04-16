// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStruct.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_struct__rosidl_typesupport_introspection_c.h"
#include "autoware_adapi_v1_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_struct__functions.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_struct__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `id`
#include "rosidl_runtime_c/string_functions.h"
// Member `nodes`
#include "autoware_adapi_v1_msgs/msg/diag_node_struct.h"
// Member `nodes`
#include "autoware_adapi_v1_msgs/msg/detail/diag_node_struct__rosidl_typesupport_introspection_c.h"
// Member `diags`
#include "autoware_adapi_v1_msgs/msg/diag_leaf_struct.h"
// Member `diags`
#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_struct__rosidl_typesupport_introspection_c.h"
// Member `links`
#include "autoware_adapi_v1_msgs/msg/diag_link_struct.h"
// Member `links`
#include "autoware_adapi_v1_msgs/msg/detail/diag_link_struct__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__init(message_memory);
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_fini_function(void * message_memory)
{
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__fini(message_memory);
}

size_t autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__size_function__DiagGraphStruct__nodes(
  const void * untyped_member)
{
  const autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence *)(untyped_member);
  return member->size;
}

const void * autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__nodes(
  const void * untyped_member, size_t index)
{
  const autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__nodes(
  void * untyped_member, size_t index)
{
  autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence * member =
    (autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence *)(untyped_member);
  return &member->data[index];
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__fetch_function__DiagGraphStruct__nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autoware_adapi_v1_msgs__msg__DiagNodeStruct * item =
    ((const autoware_adapi_v1_msgs__msg__DiagNodeStruct *)
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__nodes(untyped_member, index));
  autoware_adapi_v1_msgs__msg__DiagNodeStruct * value =
    (autoware_adapi_v1_msgs__msg__DiagNodeStruct *)(untyped_value);
  *value = *item;
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__assign_function__DiagGraphStruct__nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autoware_adapi_v1_msgs__msg__DiagNodeStruct * item =
    ((autoware_adapi_v1_msgs__msg__DiagNodeStruct *)
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__nodes(untyped_member, index));
  const autoware_adapi_v1_msgs__msg__DiagNodeStruct * value =
    (const autoware_adapi_v1_msgs__msg__DiagNodeStruct *)(untyped_value);
  *item = *value;
}

bool autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__resize_function__DiagGraphStruct__nodes(
  void * untyped_member, size_t size)
{
  autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence * member =
    (autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence *)(untyped_member);
  autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__fini(member);
  return autoware_adapi_v1_msgs__msg__DiagNodeStruct__Sequence__init(member, size);
}

size_t autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__size_function__DiagGraphStruct__diags(
  const void * untyped_member)
{
  const autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence *)(untyped_member);
  return member->size;
}

const void * autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__diags(
  const void * untyped_member, size_t index)
{
  const autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__diags(
  void * untyped_member, size_t index)
{
  autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence * member =
    (autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence *)(untyped_member);
  return &member->data[index];
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__fetch_function__DiagGraphStruct__diags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autoware_adapi_v1_msgs__msg__DiagLeafStruct * item =
    ((const autoware_adapi_v1_msgs__msg__DiagLeafStruct *)
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__diags(untyped_member, index));
  autoware_adapi_v1_msgs__msg__DiagLeafStruct * value =
    (autoware_adapi_v1_msgs__msg__DiagLeafStruct *)(untyped_value);
  *value = *item;
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__assign_function__DiagGraphStruct__diags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autoware_adapi_v1_msgs__msg__DiagLeafStruct * item =
    ((autoware_adapi_v1_msgs__msg__DiagLeafStruct *)
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__diags(untyped_member, index));
  const autoware_adapi_v1_msgs__msg__DiagLeafStruct * value =
    (const autoware_adapi_v1_msgs__msg__DiagLeafStruct *)(untyped_value);
  *item = *value;
}

bool autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__resize_function__DiagGraphStruct__diags(
  void * untyped_member, size_t size)
{
  autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence * member =
    (autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence *)(untyped_member);
  autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__fini(member);
  return autoware_adapi_v1_msgs__msg__DiagLeafStruct__Sequence__init(member, size);
}

size_t autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__size_function__DiagGraphStruct__links(
  const void * untyped_member)
{
  const autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence *)(untyped_member);
  return member->size;
}

const void * autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__links(
  const void * untyped_member, size_t index)
{
  const autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__links(
  void * untyped_member, size_t index)
{
  autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence * member =
    (autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence *)(untyped_member);
  return &member->data[index];
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__fetch_function__DiagGraphStruct__links(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autoware_adapi_v1_msgs__msg__DiagLinkStruct * item =
    ((const autoware_adapi_v1_msgs__msg__DiagLinkStruct *)
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__links(untyped_member, index));
  autoware_adapi_v1_msgs__msg__DiagLinkStruct * value =
    (autoware_adapi_v1_msgs__msg__DiagLinkStruct *)(untyped_value);
  *value = *item;
}

void autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__assign_function__DiagGraphStruct__links(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autoware_adapi_v1_msgs__msg__DiagLinkStruct * item =
    ((autoware_adapi_v1_msgs__msg__DiagLinkStruct *)
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__links(untyped_member, index));
  const autoware_adapi_v1_msgs__msg__DiagLinkStruct * value =
    (const autoware_adapi_v1_msgs__msg__DiagLinkStruct *)(untyped_value);
  *item = *value;
}

bool autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__resize_function__DiagGraphStruct__links(
  void * untyped_member, size_t size)
{
  autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence * member =
    (autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence *)(untyped_member);
  autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__fini(member);
  return autoware_adapi_v1_msgs__msg__DiagLinkStruct__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_member_array[5] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagGraphStruct, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagGraphStruct, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagGraphStruct, nodes),  // bytes offset in struct
    NULL,  // default value
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__size_function__DiagGraphStruct__nodes,  // size() function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__nodes,  // get_const(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__nodes,  // get(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__fetch_function__DiagGraphStruct__nodes,  // fetch(index, &value) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__assign_function__DiagGraphStruct__nodes,  // assign(index, value) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__resize_function__DiagGraphStruct__nodes  // resize(index) function pointer
  },
  {
    "diags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagGraphStruct, diags),  // bytes offset in struct
    NULL,  // default value
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__size_function__DiagGraphStruct__diags,  // size() function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__diags,  // get_const(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__diags,  // get(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__fetch_function__DiagGraphStruct__diags,  // fetch(index, &value) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__assign_function__DiagGraphStruct__diags,  // assign(index, value) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__resize_function__DiagGraphStruct__diags  // resize(index) function pointer
  },
  {
    "links",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagGraphStruct, links),  // bytes offset in struct
    NULL,  // default value
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__size_function__DiagGraphStruct__links,  // size() function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_const_function__DiagGraphStruct__links,  // get_const(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__get_function__DiagGraphStruct__links,  // get(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__fetch_function__DiagGraphStruct__links,  // fetch(index, &value) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__assign_function__DiagGraphStruct__links,  // assign(index, value) function pointer
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__resize_function__DiagGraphStruct__links  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_members = {
  "autoware_adapi_v1_msgs__msg",  // message namespace
  "DiagGraphStruct",  // message name
  5,  // number of fields
  sizeof(autoware_adapi_v1_msgs__msg__DiagGraphStruct),
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_member_array,  // message members
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_init_function,  // function to initialize message memory (memory has to be allocated)
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_type_support_handle = {
  0,
  &autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autoware_adapi_v1_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_adapi_v1_msgs, msg, DiagGraphStruct)() {
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_adapi_v1_msgs, msg, DiagNodeStruct)();
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_adapi_v1_msgs, msg, DiagLeafStruct)();
  autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_adapi_v1_msgs, msg, DiagLinkStruct)();
  if (!autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_type_support_handle.typesupport_identifier) {
    autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autoware_adapi_v1_msgs__msg__DiagGraphStruct__rosidl_typesupport_introspection_c__DiagGraphStruct_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
