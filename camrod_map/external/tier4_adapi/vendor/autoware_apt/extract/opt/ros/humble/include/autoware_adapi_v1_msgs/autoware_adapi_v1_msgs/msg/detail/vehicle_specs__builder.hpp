// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/VehicleSpecs.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_SPECS__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_SPECS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/vehicle_specs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleSpecs_max_steering_tire_angle
{
public:
  Init_VehicleSpecs_max_steering_tire_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_adapi_v1_msgs::msg::VehicleSpecs max_steering_tire_angle(::autoware_adapi_v1_msgs::msg::VehicleSpecs::_max_steering_tire_angle_type arg)
  {
    msg_.max_steering_tire_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::VehicleSpecs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::VehicleSpecs>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_VehicleSpecs_max_steering_tire_angle();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_SPECS__BUILDER_HPP_
