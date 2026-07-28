#pragma once

// HH_260728 - Preserve configured component/mount metadata while diagnostics
// move through the aggregator, and render the most useful checker values in
// the operator-facing SYSTEM summary.

#include <algorithm>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace camrod_system::diagnostic_detail
{

inline std::string valueFor(
  const std::vector<diagnostic_msgs::msg::KeyValue> & values,
  const std::string & key)
{
  for (const auto & value : values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "";
}

inline void upsertValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  if (key.empty() || value.empty()) {
    return;
  }
  for (auto & entry : status.values) {
    if (entry.key == key) {
      entry.value = value;
      return;
    }
  }
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = value;
  status.values.push_back(entry);
}

inline void appendField(
  std::vector<std::string> & fields,
  const std::string & label,
  const std::string & value,
  const std::string & suffix = "")
{
  if (!value.empty()) {
    fields.push_back(label + "=" + value + suffix);
  }
}

inline std::string joinFields(const std::vector<std::string> & fields)
{
  std::string output;
  for (std::size_t index = 0; index < fields.size(); ++index) {
    if (index > 0U) {
      output += ", ";
    }
    output += fields[index];
  }
  return output;
}

inline std::string formatValues(
  const std::vector<diagnostic_msgs::msg::KeyValue> & values)
{
  std::vector<std::string> fields;
  appendField(fields, "component", valueFor(values, "component_id"));
  appendField(fields, "location", valueFor(values, "sensor_location"));
  appendField(fields, "frame", valueFor(values, "sensor_frame"));
  appendField(fields, "mount_xyz_m", valueFor(values, "mount_xyz_m"));
  appendField(fields, "mount_rpy_deg", valueFor(values, "mount_rpy_deg"));
  appendField(fields, "pose_verified", valueFor(values, "pose_verified"));

  appendField(fields, "range", valueFor(values, "range_m"), "m");

  const auto actual_hz = valueFor(values, "actual_hz");
  const auto expected_hz = valueFor(values, "expected_hz");
  if (!actual_hz.empty() || !expected_hz.empty()) {
    fields.push_back(
      "rate=" + (actual_hz.empty() ? "?" : actual_hz) + "/" +
      (expected_hz.empty() ? "?" : expected_hz) + "Hz");
  }

  const auto actual_fps = valueFor(values, "actual_fps");
  const auto expected_fps = valueFor(values, "expected_fps");
  if (!actual_fps.empty() || !expected_fps.empty()) {
    fields.push_back(
      "fps=" + (actual_fps.empty() ? "?" : actual_fps) + "/" +
      (expected_fps.empty() ? "?" : expected_fps));
  }

  appendField(fields, "fix_status", valueFor(values, "fix_status"));
  appendField(fields, "points", valueFor(values, "point_count"));
  appendField(fields, "nan_ratio", valueFor(values, "nan_ratio"));
  appendField(fields, "accel_norm", valueFor(values, "accel_magnitude (m/s^2)"), "m/s^2");
  appendField(fields, "camera_info", valueFor(values, "camera_info"));
  appendField(fields, "velocity_input", valueFor(values, "velocity_input"));
  appendField(fields, "imu_input", valueFor(values, "imu_input"));
  appendField(fields, "output", valueFor(values, "output"));
  appendField(fields, "age", valueFor(values, "last_msg_sec_ago"), "s");
  appendField(fields, "topic", valueFor(values, "topic"));
  return joinFields(fields);
}

}  // namespace camrod_system::diagnostic_detail
