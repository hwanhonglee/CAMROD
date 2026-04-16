// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:srv/ListManualControlMode.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__LIST_MANUAL_CONTROL_MODE__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__LIST_MANUAL_CONTROL_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Request __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListManualControlMode_Request_
{
  using Type = ListManualControlMode_Request_<ContainerAllocator>;

  explicit ListManualControlMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ListManualControlMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Request
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Request
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListManualControlMode_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListManualControlMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListManualControlMode_Request_

// alias to use template instance with default allocator
using ListManualControlMode_Request =
  autoware_adapi_v1_msgs::srv::ListManualControlMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs


// Include directives for member types
// Member 'status'
#include "autoware_adapi_v1_msgs/msg/detail/response_status__struct.hpp"
// Member 'modes'
#include "autoware_adapi_v1_msgs/msg/detail/manual_control_mode__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Response __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListManualControlMode_Response_
{
  using Type = ListManualControlMode_Response_<ContainerAllocator>;

  explicit ListManualControlMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_init)
  {
    (void)_init;
  }

  explicit ListManualControlMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _status_type =
    autoware_adapi_v1_msgs::msg::ResponseStatus_<ContainerAllocator>;
  _status_type status;
  using _modes_type =
    std::vector<autoware_adapi_v1_msgs::msg::ManualControlMode_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::ManualControlMode_<ContainerAllocator>>>;
  _modes_type modes;

  // setters for named parameter idiom
  Type & set__status(
    const autoware_adapi_v1_msgs::msg::ResponseStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__modes(
    const std::vector<autoware_adapi_v1_msgs::msg::ManualControlMode_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::ManualControlMode_<ContainerAllocator>>> & _arg)
  {
    this->modes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Response
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__srv__ListManualControlMode_Response
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListManualControlMode_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->modes != other.modes) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListManualControlMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListManualControlMode_Response_

// alias to use template instance with default allocator
using ListManualControlMode_Response =
  autoware_adapi_v1_msgs::srv::ListManualControlMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs

namespace autoware_adapi_v1_msgs
{

namespace srv
{

struct ListManualControlMode
{
  using Request = autoware_adapi_v1_msgs::srv::ListManualControlMode_Request;
  using Response = autoware_adapi_v1_msgs::srv::ListManualControlMode_Response;
};

}  // namespace srv

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__SRV__DETAIL__LIST_MANUAL_CONTROL_MODE__STRUCT_HPP_
