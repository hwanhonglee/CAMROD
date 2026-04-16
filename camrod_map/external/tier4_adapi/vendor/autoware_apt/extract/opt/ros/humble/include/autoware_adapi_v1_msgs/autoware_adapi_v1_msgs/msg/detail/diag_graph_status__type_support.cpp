// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DiagGraphStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autoware_adapi_v1_msgs::msg::DiagGraphStatus(_init);
}

void DiagGraphStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autoware_adapi_v1_msgs::msg::DiagGraphStatus *>(message_memory);
  typed_message->~DiagGraphStatus();
}

size_t size_function__DiagGraphStatus__nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStatus> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DiagGraphStatus__nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStatus> *>(untyped_member);
  return &member[index];
}

void * get_function__DiagGraphStatus__nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStatus> *>(untyped_member);
  return &member[index];
}

void fetch_function__DiagGraphStatus__nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagNodeStatus *>(
    get_const_function__DiagGraphStatus__nodes(untyped_member, index));
  auto & value = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagNodeStatus *>(untyped_value);
  value = item;
}

void assign_function__DiagGraphStatus__nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagNodeStatus *>(
    get_function__DiagGraphStatus__nodes(untyped_member, index));
  const auto & value = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagNodeStatus *>(untyped_value);
  item = value;
}

void resize_function__DiagGraphStatus__nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStatus> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DiagGraphStatus__diags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStatus> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DiagGraphStatus__diags(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStatus> *>(untyped_member);
  return &member[index];
}

void * get_function__DiagGraphStatus__diags(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStatus> *>(untyped_member);
  return &member[index];
}

void fetch_function__DiagGraphStatus__diags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagLeafStatus *>(
    get_const_function__DiagGraphStatus__diags(untyped_member, index));
  auto & value = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagLeafStatus *>(untyped_value);
  value = item;
}

void assign_function__DiagGraphStatus__diags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagLeafStatus *>(
    get_function__DiagGraphStatus__diags(untyped_member, index));
  const auto & value = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagLeafStatus *>(untyped_value);
  item = value;
}

void resize_function__DiagGraphStatus__diags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStatus> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DiagGraphStatus_message_member_array[4] = {
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStatus, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStatus, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagNodeStatus>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStatus, nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__DiagGraphStatus__nodes,  // size() function pointer
    get_const_function__DiagGraphStatus__nodes,  // get_const(index) function pointer
    get_function__DiagGraphStatus__nodes,  // get(index) function pointer
    fetch_function__DiagGraphStatus__nodes,  // fetch(index, &value) function pointer
    assign_function__DiagGraphStatus__nodes,  // assign(index, value) function pointer
    resize_function__DiagGraphStatus__nodes  // resize(index) function pointer
  },
  {
    "diags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagLeafStatus>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStatus, diags),  // bytes offset in struct
    nullptr,  // default value
    size_function__DiagGraphStatus__diags,  // size() function pointer
    get_const_function__DiagGraphStatus__diags,  // get_const(index) function pointer
    get_function__DiagGraphStatus__diags,  // get(index) function pointer
    fetch_function__DiagGraphStatus__diags,  // fetch(index, &value) function pointer
    assign_function__DiagGraphStatus__diags,  // assign(index, value) function pointer
    resize_function__DiagGraphStatus__diags  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DiagGraphStatus_message_members = {
  "autoware_adapi_v1_msgs::msg",  // message namespace
  "DiagGraphStatus",  // message name
  4,  // number of fields
  sizeof(autoware_adapi_v1_msgs::msg::DiagGraphStatus),
  DiagGraphStatus_message_member_array,  // message members
  DiagGraphStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  DiagGraphStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DiagGraphStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DiagGraphStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagGraphStatus>()
{
  return &::autoware_adapi_v1_msgs::msg::rosidl_typesupport_introspection_cpp::DiagGraphStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autoware_adapi_v1_msgs, msg, DiagGraphStatus)() {
  return &::autoware_adapi_v1_msgs::msg::rosidl_typesupport_introspection_cpp::DiagGraphStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
