// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/MrmRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__MrmRequest __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__MrmRequest __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MrmRequest_
{
  using Type = MrmRequest_<ContainerAllocator>;

  explicit MrmRequest_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sender = "";
      this->strategy = 0;
    }
  }

  explicit MrmRequest_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sender(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sender = "";
      this->strategy = 0;
    }
  }

  // field types and members
  using _sender_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sender_type sender;
  using _strategy_type =
    uint16_t;
  _strategy_type strategy;

  // setters for named parameter idiom
  Type & set__sender(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sender = _arg;
    return *this;
  }
  Type & set__strategy(
    const uint16_t & _arg)
  {
    this->strategy = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t UNKNOWN =
    0u;
  static constexpr uint16_t CANCEL =
    1u;
  static constexpr uint16_t DELEGATE =
    2u;

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__MrmRequest
    std::shared_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__MrmRequest
    std::shared_ptr<autoware_adapi_v1_msgs::msg::MrmRequest_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MrmRequest_ & other) const
  {
    if (this->sender != other.sender) {
      return false;
    }
    if (this->strategy != other.strategy) {
      return false;
    }
    return true;
  }
  bool operator!=(const MrmRequest_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MrmRequest_

// alias to use template instance with default allocator
using MrmRequest =
  autoware_adapi_v1_msgs::msg::MrmRequest_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t MrmRequest_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t MrmRequest_<ContainerAllocator>::CANCEL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t MrmRequest_<ContainerAllocator>::DELEGATE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__MRM_REQUEST__STRUCT_HPP_
