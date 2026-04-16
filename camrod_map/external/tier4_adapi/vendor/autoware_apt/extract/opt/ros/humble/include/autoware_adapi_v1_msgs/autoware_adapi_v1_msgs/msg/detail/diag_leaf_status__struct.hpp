// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/DiagLeafStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'values'
#include "autoware_adapi_v1_msgs/msg/detail/kv_string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLeafStatus __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLeafStatus __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DiagLeafStatus_
{
  using Type = DiagLeafStatus_<ContainerAllocator>;

  explicit DiagLeafStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->level = 0;
      this->input_level = 0;
      this->message = "";
      this->hardware_id = "";
    }
  }

  explicit DiagLeafStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    hardware_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->level = 0;
      this->input_level = 0;
      this->message = "";
      this->hardware_id = "";
    }
  }

  // field types and members
  using _level_type =
    unsigned char;
  _level_type level;
  using _input_level_type =
    unsigned char;
  _input_level_type input_level;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _hardware_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _hardware_id_type hardware_id;
  using _values_type =
    std::vector<autoware_adapi_v1_msgs::msg::KvString_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::KvString_<ContainerAllocator>>>;
  _values_type values;

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
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__hardware_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->hardware_id = _arg;
    return *this;
  }
  Type & set__values(
    const std::vector<autoware_adapi_v1_msgs::msg::KvString_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autoware_adapi_v1_msgs::msg::KvString_<ContainerAllocator>>> & _arg)
  {
    this->values = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLeafStatus
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__DiagLeafStatus
    std::shared_ptr<autoware_adapi_v1_msgs::msg::DiagLeafStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DiagLeafStatus_ & other) const
  {
    if (this->level != other.level) {
      return false;
    }
    if (this->input_level != other.input_level) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->hardware_id != other.hardware_id) {
      return false;
    }
    if (this->values != other.values) {
      return false;
    }
    return true;
  }
  bool operator!=(const DiagLeafStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DiagLeafStatus_

// alias to use template instance with default allocator
using DiagLeafStatus =
  autoware_adapi_v1_msgs::msg::DiagLeafStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__DIAG_LEAF_STATUS__STRUCT_HPP_
