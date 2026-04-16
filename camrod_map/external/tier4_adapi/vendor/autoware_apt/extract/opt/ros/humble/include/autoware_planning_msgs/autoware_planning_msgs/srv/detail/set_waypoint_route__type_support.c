// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autoware_planning_msgs:srv/SetWaypointRoute.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autoware_planning_msgs/srv/detail/set_waypoint_route__rosidl_typesupport_introspection_c.h"
#include "autoware_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autoware_planning_msgs/srv/detail/set_waypoint_route__functions.h"
#include "autoware_planning_msgs/srv/detail/set_waypoint_route__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `goal_pose`
// Member `waypoints`
#include "geometry_msgs/msg/pose.h"
// Member `goal_pose`
// Member `waypoints`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `uuid`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `uuid`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autoware_planning_msgs__srv__SetWaypointRoute_Request__init(message_memory);
}

void autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_fini_function(void * message_memory)
{
  autoware_planning_msgs__srv__SetWaypointRoute_Request__fini(message_memory);
}

size_t autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__size_function__SetWaypointRoute_Request__waypoints(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__get_const_function__SetWaypointRoute_Request__waypoints(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__get_function__SetWaypointRoute_Request__waypoints(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__fetch_function__SetWaypointRoute_Request__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__get_const_function__SetWaypointRoute_Request__waypoints(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__assign_function__SetWaypointRoute_Request__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__get_function__SetWaypointRoute_Request__waypoints(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__resize_function__SetWaypointRoute_Request__waypoints(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_planning_msgs__srv__SetWaypointRoute_Request, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_planning_msgs__srv__SetWaypointRoute_Request, goal_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_planning_msgs__srv__SetWaypointRoute_Request, waypoints),  // bytes offset in struct
    NULL,  // default value
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__size_function__SetWaypointRoute_Request__waypoints,  // size() function pointer
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__get_const_function__SetWaypointRoute_Request__waypoints,  // get_const(index) function pointer
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__get_function__SetWaypointRoute_Request__waypoints,  // get(index) function pointer
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__fetch_function__SetWaypointRoute_Request__waypoints,  // fetch(index, &value) function pointer
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__assign_function__SetWaypointRoute_Request__waypoints,  // assign(index, value) function pointer
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__resize_function__SetWaypointRoute_Request__waypoints  // resize(index) function pointer
  },
  {
    "uuid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_planning_msgs__srv__SetWaypointRoute_Request, uuid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "allow_modification",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_planning_msgs__srv__SetWaypointRoute_Request, allow_modification),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_members = {
  "autoware_planning_msgs__srv",  // message namespace
  "SetWaypointRoute_Request",  // message name
  5,  // number of fields
  sizeof(autoware_planning_msgs__srv__SetWaypointRoute_Request),
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_member_array,  // message members
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_type_support_handle = {
  0,
  &autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autoware_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute_Request)() {
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_type_support_handle.typesupport_identifier) {
    autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autoware_planning_msgs__srv__SetWaypointRoute_Request__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "autoware_planning_msgs/srv/detail/set_waypoint_route__rosidl_typesupport_introspection_c.h"
// already included above
// #include "autoware_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "autoware_planning_msgs/srv/detail/set_waypoint_route__functions.h"
// already included above
// #include "autoware_planning_msgs/srv/detail/set_waypoint_route__struct.h"


// Include directives for member types
// Member `status`
#include "autoware_common_msgs/msg/response_status.h"
// Member `status`
#include "autoware_common_msgs/msg/detail/response_status__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autoware_planning_msgs__srv__SetWaypointRoute_Response__init(message_memory);
}

void autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_fini_function(void * message_memory)
{
  autoware_planning_msgs__srv__SetWaypointRoute_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_member_array[1] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_planning_msgs__srv__SetWaypointRoute_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_members = {
  "autoware_planning_msgs__srv",  // message namespace
  "SetWaypointRoute_Response",  // message name
  1,  // number of fields
  sizeof(autoware_planning_msgs__srv__SetWaypointRoute_Response),
  autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_member_array,  // message members
  autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_type_support_handle = {
  0,
  &autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autoware_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute_Response)() {
  autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_common_msgs, msg, ResponseStatus)();
  if (!autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_type_support_handle.typesupport_identifier) {
    autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autoware_planning_msgs__srv__SetWaypointRoute_Response__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autoware_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "autoware_planning_msgs/srv/detail/set_waypoint_route__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_members = {
  "autoware_planning_msgs__srv",  // service namespace
  "SetWaypointRoute",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_Request_message_type_support_handle,
  NULL  // response message
  // autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_Response_message_type_support_handle
};

static rosidl_service_type_support_t autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_type_support_handle = {
  0,
  &autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autoware_planning_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute)() {
  if (!autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_type_support_handle.typesupport_identifier) {
    autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_planning_msgs, srv, SetWaypointRoute_Response)()->data;
  }

  return &autoware_planning_msgs__srv__detail__set_waypoint_route__rosidl_typesupport_introspection_c__SetWaypointRoute_service_type_support_handle;
}
