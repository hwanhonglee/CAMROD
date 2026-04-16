// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_planning_msgs:srv/SetPreferredPrimitive.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__STRUCT_HPP_
#define AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'preferred_primitives'
#include "autoware_planning_msgs/msg/detail/lanelet_primitive__struct.hpp"
// Member 'uuid'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Request __attribute__((deprecated))
#else
# define DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Request __declspec(deprecated)
#endif

namespace autoware_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetPreferredPrimitive_Request_
{
  using Type = SetPreferredPrimitive_Request_<ContainerAllocator>;

  explicit SetPreferredPrimitive_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uuid(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reset = false;
    }
  }

  explicit SetPreferredPrimitive_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uuid(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reset = false;
    }
  }

  // field types and members
  using _preferred_primitives_type =
    std::vector<autoware_planning_msgs::msg::LaneletPrimitive_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_planning_msgs::msg::LaneletPrimitive_<ContainerAllocator>>>;
  _preferred_primitives_type preferred_primitives;
  using _reset_type =
    bool;
  _reset_type reset;
  using _uuid_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _uuid_type uuid;

  // setters for named parameter idiom
  Type & set__preferred_primitives(
    const std::vector<autoware_planning_msgs::msg::LaneletPrimitive_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_planning_msgs::msg::LaneletPrimitive_<ContainerAllocator>>> & _arg)
  {
    this->preferred_primitives = _arg;
    return *this;
  }
  Type & set__reset(
    const bool & _arg)
  {
    this->reset = _arg;
    return *this;
  }
  Type & set__uuid(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Request
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Request
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPreferredPrimitive_Request_ & other) const
  {
    if (this->preferred_primitives != other.preferred_primitives) {
      return false;
    }
    if (this->reset != other.reset) {
      return false;
    }
    if (this->uuid != other.uuid) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPreferredPrimitive_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPreferredPrimitive_Request_

// alias to use template instance with default allocator
using SetPreferredPrimitive_Request =
  autoware_planning_msgs::srv::SetPreferredPrimitive_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_planning_msgs


// Include directives for member types
// Member 'status'
#include "autoware_common_msgs/msg/detail/response_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Response __attribute__((deprecated))
#else
# define DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Response __declspec(deprecated)
#endif

namespace autoware_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetPreferredPrimitive_Response_
{
  using Type = SetPreferredPrimitive_Response_<ContainerAllocator>;

  explicit SetPreferredPrimitive_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_init)
  {
    (void)_init;
  }

  explicit SetPreferredPrimitive_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Response
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_planning_msgs__srv__SetPreferredPrimitive_Response
    std::shared_ptr<autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPreferredPrimitive_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPreferredPrimitive_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPreferredPrimitive_Response_

// alias to use template instance with default allocator
using SetPreferredPrimitive_Response =
  autoware_planning_msgs::srv::SetPreferredPrimitive_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_planning_msgs

namespace autoware_planning_msgs
{

namespace srv
{

struct SetPreferredPrimitive
{
  using Request = autoware_planning_msgs::srv::SetPreferredPrimitive_Request;
  using Response = autoware_planning_msgs::srv::SetPreferredPrimitive_Response;
};

}  // namespace srv

}  // namespace autoware_planning_msgs

#endif  // AUTOWARE_PLANNING_MSGS__SRV__DETAIL__SET_PREFERRED_PRIMITIVE__STRUCT_HPP_
