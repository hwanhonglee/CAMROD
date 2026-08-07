#include <cassert>
#include <iostream>
#include <string>

#include "camrod_system/diagnostic_detail.hpp"

namespace
{

void add(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  camrod_system::diagnostic_detail::upsertValue(status, key, value);
}

}  // namespace

int main()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "radar_checker: /sensor/radar/LEFT2";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = "Publish rate low";
  add(status, "component_id", "radar.left2");
  add(status, "sensor_location", "left_rear");
  add(status, "sensor_frame", "radar_left2_link");
  // HH_260807 - Use the remeasured center-frame LEFT2 mount in operator detail.
  add(status, "mount_xyz_m", "-0.38,0.53,0.29013");
  add(status, "range_m", "0.223");
  add(status, "actual_hz", "5.1");
  add(status, "expected_hz", "8.0");

  // HH_260728 - Registry metadata replaces stale values by key rather than
  // duplicating them, matching aggregator behavior on every publish.
  add(status, "sensor_location", "left_rear");
  std::size_t location_count = 0U;
  for (const auto & value : status.values) {
    if (value.key == "sensor_location") {
      ++location_count;
    }
  }
  assert(location_count == 1U);

  const auto stale = [&status]() {
      auto output = status;
      output.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      output.message = "STALE (last seen 3.1s ago)";
      return output;
    }();
  assert(stale.values.size() == status.values.size());
  assert(
    camrod_system::diagnostic_detail::valueFor(
      stale.values, "sensor_location") == "left_rear");

  const auto detail = camrod_system::diagnostic_detail::formatValues(stale.values);
  assert(detail.find("component=radar.left2") != std::string::npos);
  assert(detail.find("location=left_rear") != std::string::npos);
  assert(detail.find("frame=radar_left2_link") != std::string::npos);
  assert(detail.find("range=0.223m") != std::string::npos);
  assert(detail.find("rate=5.1/8.0Hz") != std::string::npos);

  diagnostic_msgs::msg::DiagnosticStatus unknown;
  add(unknown, "actual_hz", "1.0");
  const auto unknown_detail =
    camrod_system::diagnostic_detail::formatValues(unknown.values);
  assert(unknown_detail.find("location=") == std::string::npos);
  assert(unknown_detail.find("frame=") == std::string::npos);

  std::cout << "diagnostic detail tests passed" << std::endl;
  return 0;
}
