#pragma once

// HH_260721 - Centralize control module-state and diagnostic message construction.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/module_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_control
{

inline avg_msgs::msg::ModuleState makeModuleState(
  rclcpp::Node & node,
  const std::string & module_name,
  const uint8_t level,
  const std::string & message)
{
  avg_msgs::msg::ModuleState output;
  output.stamp = node.get_clock()->now();
  output.module_name = module_name;
  output.level = level;
  output.message = message;
  return output;
}

inline diagnostic_msgs::msg::DiagnosticArray makeDiagnostics(
  rclcpp::Node & node,
  const std::string & name,
  const std::string & category,
  const uint8_t level,
  const std::string & message,
  const std::vector<std::pair<std::string, std::string>> & values = {},
  const std::string & hardware_id = "camrod_control")
{
  diagnostic_msgs::msg::DiagnosticArray output;
  output.header.stamp = node.get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = hardware_id;
  status.level = level;
  status.message = message;
  status.values.push_back(diagnostic_msgs::msg::KeyValue().set__key("category").set__value(category));
  status.values.push_back(diagnostic_msgs::msg::KeyValue().set__key("module").set__value(category));
  for (const auto & value : values) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = value.first;
    entry.value = value.second;
    status.values.push_back(entry);
  }
  output.status.push_back(status);
  return output;
}

}  // namespace camrod_control
