// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_planning_msgs:srv/SetLaneletRoute.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_LANELET_ROUTE__STRUCT_HPP_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_LANELET_ROUTE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'segments'
#include "autoware_planning_msgs/msg/detail/lanelet_segment__struct.hpp"
// Member 'uuid'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Request __attribute__((deprecated))
#else
# define DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Request __declspec(deprecated)
#endif

namespace autoware_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLaneletRoute_Request_
{
  using Type = SetLaneletRoute_Request_<ContainerAllocator>;

  explicit SetLaneletRoute_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    goal_pose(_init),
    uuid(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->allow_modification = false;
    }
  }

  explicit SetLaneletRoute_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    goal_pose(_alloc, _init),
    uuid(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->allow_modification = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _goal_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _goal_pose_type goal_pose;
  using _segments_type =
    std::vector<autoware_planning_msgs::msg::LaneletSegment_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_planning_msgs::msg::LaneletSegment_<ContainerAllocator>>>;
  _segments_type segments;
  using _uuid_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _uuid_type uuid;
  using _allow_modification_type =
    bool;
  _allow_modification_type allow_modification;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__goal_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->goal_pose = _arg;
    return *this;
  }
  Type & set__segments(
    const std::vector<autoware_planning_msgs::msg::LaneletSegment_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_planning_msgs::msg::LaneletSegment_<ContainerAllocator>>> & _arg)
  {
    this->segments = _arg;
    return *this;
  }
  Type & set__uuid(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }
  Type & set__allow_modification(
    const bool & _arg)
  {
    this->allow_modification = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Request
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Request
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLaneletRoute_Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->goal_pose != other.goal_pose) {
      return false;
    }
    if (this->segments != other.segments) {
      return false;
    }
    if (this->uuid != other.uuid) {
      return false;
    }
    if (this->allow_modification != other.allow_modification) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLaneletRoute_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLaneletRoute_Request_

// alias to use template instance with default allocator
using SetLaneletRoute_Request =
  autoware_planning_msgs::srv::SetLaneletRoute_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_planning_msgs


// Include directives for member types
// Member 'status'
#include "autoware_common_msgs/msg/detail/response_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Response __attribute__((deprecated))
#else
# define DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Response __declspec(deprecated)
#endif

namespace autoware_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLaneletRoute_Response_
{
  using Type = SetLaneletRoute_Response_<ContainerAllocator>;

  explicit SetLaneletRoute_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_init)
  {
    (void)_init;
  }

  explicit SetLaneletRoute_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _status_type =
    autoware_common_msgs::msg::ResponseStatus_<ContainerAllocator>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const autoware_common_msgs::msg::ResponseStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Response
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_planning_msgs__srv__SetLaneletRoute_Response
    std::shared_ptr<autoware_planning_msgs::srv::SetLaneletRoute_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLaneletRoute_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLaneletRoute_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLaneletRoute_Response_

// alias to use template instance with default allocator
using SetLaneletRoute_Response =
  autoware_planning_msgs::srv::SetLaneletRoute_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_planning_msgs

namespace autoware_planning_msgs
{

namespace srv
{

struct SetLaneletRoute
{
  using Request = autoware_planning_msgs::srv::SetLaneletRoute_Request;
  using Response = autoware_planning_msgs::srv::SetLaneletRoute_Response;
};

}  // namespace srv

}  // namespace autoware_planning_msgs

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_LANELET_ROUTE__STRUCT_HPP_
