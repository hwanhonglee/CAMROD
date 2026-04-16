// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagLinkStruct.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLinkStruct __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLinkStruct __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DiagLinkStruct_
{
  using Type = DiagLinkStruct_<ContainerAllocator>;

  explicit DiagLinkStruct_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->parent = 0ul;
      this->child = 0ul;
    }
  }

  explicit DiagLinkStruct_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->parent = 0ul;
      this->child = 0ul;
    }
  }

  // field types and members
  using _parent_type =
    uint32_t;
  _parent_type parent;
  using _child_type =
    uint32_t;
  _child_type child;

  // setters for named parameter idiom
  Type & set__parent(
    const uint32_t & _arg)
  {
    this->parent = _arg;
    return *this;
  }
  Type & set__child(
    const uint32_t & _arg)
  {
    this->child = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLinkStruct
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLinkStruct
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLinkStruct_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DiagLinkStruct_ & other) const
  {
    if (this->parent != other.parent) {
      return false;
    }
    if (this->child != other.child) {
      return false;
    }
    return true;
  }
  bool operator!=(const DiagLinkStruct_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DiagLinkStruct_

// alias to use template instance with default allocator
using DiagLinkStruct =
  autoware_adapi_v1_msgs::msg::DiagLinkStruct_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LINK_STRUCT__STRUCT_HPP_
