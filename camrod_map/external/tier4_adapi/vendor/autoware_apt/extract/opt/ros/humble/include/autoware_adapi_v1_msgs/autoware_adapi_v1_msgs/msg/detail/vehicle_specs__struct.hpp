// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_adapi_v1_msgs:msg/VehicleSpecs.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_SPECS__STRUCT_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_SPECS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autoware_adapi_v1_msgs__msg__VehicleSpecs __attribute__((deprecated))
#else
# define DEPRECATED__autoware_adapi_v1_msgs__msg__VehicleSpecs __declspec(deprecated)
#endif

namespace autoware_adapi_v1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleSpecs_
{
  using Type = VehicleSpecs_<ContainerAllocator>;

  explicit VehicleSpecs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_steering_tire_angle = 0.0f;
    }
  }

  explicit VehicleSpecs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_steering_tire_angle = 0.0f;
    }
  }

  // field types and members
  using _max_steering_tire_angle_type =
    float;
  _max_steering_tire_angle_type max_steering_tire_angle;

  // setters for named parameter idiom
  Type & set__max_steering_tire_angle(
    const float & _arg)
  {
    this->max_steering_tire_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__VehicleSpecs
    std::shared_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_adapi_v1_msgs__msg__VehicleSpecs
    std::shared_ptr<autoware_adapi_v1_msgs::msg::VehicleSpecs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleSpecs_ & other) const
  {
    if (this->max_steering_tire_angle != other.max_steering_tire_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleSpecs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleSpecs_

// alias to use template instance with default allocator
using VehicleSpecs =
  autoware_adapi_v1_msgs::msg::VehicleSpecs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_SPECS__STRUCT_HPP_
