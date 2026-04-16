// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/TurnIndicatorsCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__TURN_INDICATORS_COMMAND__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__TURN_INDICATORS_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'command'
#include "autoware_adapi_v1_msgs/msg/detail/turn_indicators__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TurnIndicatorsCommand_
{
  using Type = TurnIndicatorsCommand_<ContainerAllocator>;

  explicit TurnIndicatorsCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    command(_init)
  {
    (void)_init;
  }

  explicit TurnIndicatorsCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    command(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _command_type =
    autoware_adapi_v1_msgs::msg::TurnIndicators_<ContainerAllocator>;
  _command_type command;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__command(
    const autoware_adapi_v1_msgs::msg::TurnIndicators_<ContainerAllocator> & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand
    std::shared_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__TurnIndicatorsCommand
    std::shared_ptr<autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TurnIndicatorsCommand_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const TurnIndicatorsCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TurnIndicatorsCommand_

// alias to use template instance with default allocator
using TurnIndicatorsCommand =
  autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__TURN_INDICATORS_COMMAND__STRUCT_HPP_
