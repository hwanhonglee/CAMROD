// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagNodeStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagNodeStatus __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagNodeStatus __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DiagNodeStatus_
{
  using Type = DiagNodeStatus_<ContainerAllocator>;

  explicit DiagNodeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->level = 0;
      this->input_level = 0;
      this->latch_level = 0;
      this->is_dependent = false;
    }
  }

  explicit DiagNodeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->level = 0;
      this->input_level = 0;
      this->latch_level = 0;
      this->is_dependent = false;
    }
  }

  // field types and members
  using _level_type =
    unsigned char;
  _level_type level;
  using _input_level_type =
    unsigned char;
  _input_level_type input_level;
  using _latch_level_type =
    unsigned char;
  _latch_level_type latch_level;
  using _is_dependent_type =
    bool;
  _is_dependent_type is_dependent;

  // setters for named parameter idiom
  Type & set__level(
    const unsigned char & _arg)
  {
    this->level = _arg;
    return *this;
  }
  Type & set__input_level(
    const unsigned char & _arg)
  {
    this->input_level = _arg;
    return *this;
  }
  Type & set__latch_level(
    const unsigned char & _arg)
  {
    this->latch_level = _arg;
    return *this;
  }
  Type & set__is_dependent(
    const bool & _arg)
  {
    this->is_dependent = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagNodeStatus
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagNodeStatus
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagNodeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DiagNodeStatus_ & other) const
  {
    if (this->level != other.level) {
      return false;
    }
    if (this->input_level != other.input_level) {
      return false;
    }
    if (this->latch_level != other.latch_level) {
      return false;
    }
    if (this->is_dependent != other.is_dependent) {
      return false;
    }
    return true;
  }
  bool operator!=(const DiagNodeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DiagNodeStatus_

// alias to use template instance with default allocator
using DiagNodeStatus =
  autoware_adapi_v1_msgs::msg::DiagNodeStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_NODE_STATUS__STRUCT_HPP_
