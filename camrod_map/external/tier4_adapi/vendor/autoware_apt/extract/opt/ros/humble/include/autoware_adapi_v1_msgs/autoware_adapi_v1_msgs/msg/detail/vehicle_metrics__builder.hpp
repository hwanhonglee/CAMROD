// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_adapi_v1_msgs:msg/VehicleMetrics.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_METRICS__BUILDER_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_METRICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_adapi_v1_msgs/msg/detail/vehicle_metrics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_adapi_v1_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleMetrics_energy
{
public:
  explicit Init_VehicleMetrics_energy(::autoware_adapi_v1_msgs::msg::VehicleMetrics & msg)
  : msg_(msg)
  {}
  ::autoware_adapi_v1_msgs::msg::VehicleMetrics energy(::autoware_adapi_v1_msgs::msg::VehicleMetrics::_energy_type arg)
  {
    msg_.energy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::VehicleMetrics msg_;
};

class Init_VehicleMetrics_stamp
{
public:
  Init_VehicleMetrics_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleMetrics_energy stamp(::autoware_adapi_v1_msgs::msg::VehicleMetrics::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_VehicleMetrics_energy(msg_);
  }

private:
  ::autoware_adapi_v1_msgs::msg::VehicleMetrics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_adapi_v1_msgs::msg::VehicleMetrics>()
{
  return autoware_adapi_v1_msgs::msg::builder::Init_VehicleMetrics_stamp();
}

}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__MSG__DETAIL__VEHICLE_METRICS__BUILDER_HPP_
