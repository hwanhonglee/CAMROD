// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagGraphStruct.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "autoware_adapi_v1_msgs/msg/detail/diag_graph_struct__struct.hpp"
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

void DiagGraphStruct_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) autoware_adapi_v1_msgs::msg::DiagGraphStruct(_init);
}

void DiagGraphStruct_fini_function(void * message_memory)
{
  auto typed_message = static_cast<autoware_adapi_v1_msgs::msg::DiagGraphStruct *>(message_memory);
  typed_message->~DiagGraphStruct();
}

size_t size_function__DiagGraphStruct__nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStruct> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DiagGraphStruct__nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStruct> *>(untyped_member);
  return &member[index];
}

void * get_function__DiagGraphStruct__nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStruct> *>(untyped_member);
  return &member[index];
}

void fetch_function__DiagGraphStruct__nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagNodeStruct *>(
    get_const_function__DiagGraphStruct__nodes(untyped_member, index));
  auto & value = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagNodeStruct *>(untyped_value);
  value = item;
}

void assign_function__DiagGraphStruct__nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagNodeStruct *>(
    get_function__DiagGraphStruct__nodes(untyped_member, index));
  const auto & value = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagNodeStruct *>(untyped_value);
  item = value;
}

void resize_function__DiagGraphStruct__nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagNodeStruct> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DiagGraphStruct__diags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStruct> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DiagGraphStruct__diags(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStruct> *>(untyped_member);
  return &member[index];
}

void * get_function__DiagGraphStruct__diags(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStruct> *>(untyped_member);
  return &member[index];
}

void fetch_function__DiagGraphStruct__diags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagLeafStruct *>(
    get_const_function__DiagGraphStruct__diags(untyped_member, index));
  auto & value = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagLeafStruct *>(untyped_value);
  value = item;
}

void assign_function__DiagGraphStruct__diags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagLeafStruct *>(
    get_function__DiagGraphStruct__diags(untyped_member, index));
  const auto & value = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagLeafStruct *>(untyped_value);
  item = value;
}

void resize_function__DiagGraphStruct__diags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagLeafStruct> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DiagGraphStruct__links(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagLinkStruct> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DiagGraphStruct__links(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<autoware_adapi_v1_msgs::msg::DiagLinkStruct> *>(untyped_member);
  return &member[index];
}

void * get_function__DiagGraphStruct__links(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagLinkStruct> *>(untyped_member);
  return &member[index];
}

void fetch_function__DiagGraphStruct__links(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagLinkStruct *>(
    get_const_function__DiagGraphStruct__links(untyped_member, index));
  auto & value = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagLinkStruct *>(untyped_value);
  value = item;
}

void assign_function__DiagGraphStruct__links(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<autoware_adapi_v1_msgs::msg::DiagLinkStruct *>(
    get_function__DiagGraphStruct__links(untyped_member, index));
  const auto & value = *reinterpret_cast<const autoware_adapi_v1_msgs::msg::DiagLinkStruct *>(untyped_value);
  item = value;
}

void resize_function__DiagGraphStruct__links(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<autoware_adapi_v1_msgs::msg::DiagLinkStruct> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DiagGraphStruct_message_member_array[5] = {
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStruct, stamp),  // bytes offset in struct
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
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStruct, id),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagNodeStruct>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStruct, nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__DiagGraphStruct__nodes,  // size() function pointer
    get_const_function__DiagGraphStruct__nodes,  // get_const(index) function pointer
    get_function__DiagGraphStruct__nodes,  // get(index) function pointer
    fetch_function__DiagGraphStruct__nodes,  // fetch(index, &value) function pointer
    assign_function__DiagGraphStruct__nodes,  // assign(index, value) function pointer
    resize_function__DiagGraphStruct__nodes  // resize(index) function pointer
  },
  {
    "diags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagLeafStruct>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStruct, diags),  // bytes offset in struct
    nullptr,  // default value
    size_function__DiagGraphStruct__diags,  // size() function pointer
    get_const_function__DiagGraphStruct__diags,  // get_const(index) function pointer
    get_function__DiagGraphStruct__diags,  // get(index) function pointer
    fetch_function__DiagGraphStruct__diags,  // fetch(index, &value) function pointer
    assign_function__DiagGraphStruct__diags,  // assign(index, value) function pointer
    resize_function__DiagGraphStruct__diags  // resize(index) function pointer
  },
  {
    "links",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagLinkStruct>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_adapi_v1_msgs::msg::DiagGraphStruct, links),  // bytes offset in struct
    nullptr,  // default value
    size_function__DiagGraphStruct__links,  // size() function pointer
    get_const_function__DiagGraphStruct__links,  // get_const(index) function pointer
    get_function__DiagGraphStruct__links,  // get(index) function pointer
    fetch_function__DiagGraphStruct__links,  // fetch(index, &value) function pointer
    assign_function__DiagGraphStruct__links,  // assign(index, value) function pointer
    resize_function__DiagGraphStruct__links  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DiagGraphStruct_message_members = {
  "autoware_adapi_v1_msgs::msg",  // message namespace
  "DiagGraphStruct",  // message name
  5,  // number of fields
  sizeof(autoware_adapi_v1_msgs::msg::DiagGraphStruct),
  DiagGraphStruct_message_member_array,  // message members
  DiagGraphStruct_init_function,  // function to initialize message memory (memory has to be allocated)
  DiagGraphStruct_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DiagGraphStruct_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DiagGraphStruct_message_members,
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
get_message_type_support_handle<autoware_adapi_v1_msgs::msg::DiagGraphStruct>()
{
  return &::autoware_adapi_v1_msgs::msg::rosidl_typesupport_introspection_cpp::DiagGraphStruct_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autoware_adapi_v1_msgs, msg, DiagGraphStruct)() {
  return &::autoware_adapi_v1_msgs::msg::rosidl_typesupport_introspection_cpp::DiagGraphStruct_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
