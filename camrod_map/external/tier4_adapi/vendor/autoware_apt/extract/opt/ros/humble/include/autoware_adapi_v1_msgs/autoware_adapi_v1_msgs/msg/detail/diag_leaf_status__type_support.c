// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autoware_adapi_v1_msgs:msg/DiagLeafStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_status__rosidl_typesupport_introspection_c.h"
#include "autoware_adapi_v1_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_status__functions.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_leaf_status__struct.h"


// Include directives for member types
// Member `message`
// Member `hardware_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `values`
#include "autoware_adapi_v1_msgs/msg/kv_string.h"
// Member `values`
#include "autoware_adapi_v1_msgs/msg/detail/kv_string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autoware_adapi_v1_msgs__msg__DiagLeafStatus__init(message_memory);
}

void autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_fini_function(void * message_memory)
{
  autoware_adapi_v1_msgs__msg__DiagLeafStatus__fini(message_memory);
}

size_t autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__size_function__DiagLeafStatus__values(
  const void * untyped_member)
{
  const autoware_adapi_v1_msgs__msg__KvString__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__KvString__Sequence *)(untyped_member);
  return member->size;
}

const void * autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__get_const_function__DiagLeafStatus__values(
  const void * untyped_member, size_t index)
{
  const autoware_adapi_v1_msgs__msg__KvString__Sequence * member =
    (const autoware_adapi_v1_msgs__msg__KvString__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__get_function__DiagLeafStatus__values(
  void * untyped_member, size_t index)
{
  autoware_adapi_v1_msgs__msg__KvString__Sequence * member =
    (autoware_adapi_v1_msgs__msg__KvString__Sequence *)(untyped_member);
  return &member->data[index];
}

void autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__fetch_function__DiagLeafStatus__values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autoware_adapi_v1_msgs__msg__KvString * item =
    ((const autoware_adapi_v1_msgs__msg__KvString *)
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__get_const_function__DiagLeafStatus__values(untyped_member, index));
  autoware_adapi_v1_msgs__msg__KvString * value =
    (autoware_adapi_v1_msgs__msg__KvString *)(untyped_value);
  *value = *item;
}

void autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__assign_function__DiagLeafStatus__values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autoware_adapi_v1_msgs__msg__KvString * item =
    ((autoware_adapi_v1_msgs__msg__KvString *)
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__get_function__DiagLeafStatus__values(untyped_member, index));
  const autoware_adapi_v1_msgs__msg__KvString * value =
    (const autoware_adapi_v1_msgs__msg__KvString *)(untyped_value);
  *item = *value;
}

bool autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__resize_function__DiagLeafStatus__values(
  void * untyped_member, size_t size)
{
  autoware_adapi_v1_msgs__msg__KvString__Sequence * member =
    (autoware_adapi_v1_msgs__msg__KvString__Sequence *)(untyped_member);
  autoware_adapi_v1_msgs__msg__KvString__Sequence__fini(member);
  return autoware_adapi_v1_msgs__msg__KvString__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_member_array[5] = {
  {
    "level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagLeafStatus, level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "input_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagLeafStatus, input_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagLeafStatus, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hardware_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagLeafStatus, hardware_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs__msg__DiagLeafStatus, values),  // bytes offset in struct
    NULL,  // default value
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__size_function__DiagLeafStatus__values,  // size() function pointer
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__get_const_function__DiagLeafStatus__values,  // get_const(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__get_function__DiagLeafStatus__values,  // get(index) function pointer
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__fetch_function__DiagLeafStatus__values,  // fetch(index, &value) function pointer
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__assign_function__DiagLeafStatus__values,  // assign(index, value) function pointer
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__resize_function__DiagLeafStatus__values  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_members = {
  "autoware_adapi_v1_msgs__msg",  // message namespace
  "DiagLeafStatus",  // message name
  5,  // number of fields
  sizeof(autoware_adapi_v1_msgs__msg__DiagLeafStatus),
  autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_member_array,  // message members
  autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_type_support_handle = {
  0,
  &autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autoware_adapi_v1_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_adapi_v1_msgs, msg, DiagLeafStatus)() {
  autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_adapi_v1_msgs, msg, KvString)();
  if (!autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_type_support_handle.typesupport_identifier) {
    autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autoware_adapi_v1_msgs__msg__DiagLeafStatus__rosidl_typesupport_introspection_c__DiagLeafStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
