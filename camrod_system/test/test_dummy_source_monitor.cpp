#include <cassert>
#include <cmath>
#include <iostream>

#include <rclcpp/time.hpp>

#include "camrod_system/dummy_source_monitor.hpp"

namespace
{

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(
    static_cast<int64_t>(seconds * 1.0e9),
    RCL_ROS_TIME);
}

}  // namespace

int main()
{
  camrod_system::DummySourceMonitor monitor;
  double age_s = 123.0;

  // HH_260729 - Missing and explicit-false heartbeats may never hide a
  // physical sensor failure.
  assert(!monitor.isActive(at(10.0), 1.0, age_s));
  assert(age_s == -1.0);
  monitor.update(false, at(10.0));
  assert(!monitor.isActive(at(10.1), 1.0, age_s));
  assert(age_s == -1.0);

  // Fresh true is active through the configured inclusive timeout.
  monitor.update(true, at(20.0));
  assert(monitor.isActive(at(20.25), 1.0, age_s));
  assert(std::abs(age_s - 0.25) < 1.0e-9);
  assert(monitor.isActive(at(21.0), 1.0, age_s));
  assert(std::abs(age_s - 1.0) < 1.0e-9);

  // Stale, invalid-timeout, and future-dated heartbeats fail visible.
  assert(!monitor.isActive(at(21.001), 1.0, age_s));
  assert(!monitor.isActive(at(20.5), 0.0, age_s));
  assert(age_s == -1.0);
  assert(!monitor.isActive(at(19.5), 1.0, age_s));
  assert(age_s < 0.0);

  // A fresh update recovers dummy mode; a later false update revokes it
  // immediately without waiting for timeout.
  monitor.update(true, at(30.0));
  assert(monitor.isActive(at(30.1), 0.5, age_s));
  monitor.update(false, at(30.2));
  assert(!monitor.isActive(at(30.21), 0.5, age_s));
  assert(age_s == -1.0);

  // HH_260729 - Radar uses independent global and per-channel monitors.
  // Either fresh true heartbeat selects dummy mode, while an unrelated
  // channel remains fail-visible.
  camrod_system::DummySourceMonitor global_radar;
  camrod_system::DummySourceMonitor left2_radar;
  camrod_system::DummySourceMonitor right2_radar;
  double global_age_s = -1.0;
  double left2_age_s = -1.0;
  double right2_age_s = -1.0;
  left2_radar.update(true, at(40.0));
  const bool global_active =
    global_radar.isActive(at(40.1), 1.0, global_age_s);
  const bool left2_active =
    left2_radar.isActive(at(40.1), 1.0, left2_age_s);
  const bool right2_active =
    right2_radar.isActive(at(40.1), 1.0, right2_age_s);
  assert(global_active || left2_active);
  assert(!(global_active || right2_active));

  global_radar.update(true, at(41.0));
  assert(
    global_radar.isActive(at(41.1), 1.0, global_age_s) ||
    right2_radar.isActive(at(41.1), 1.0, right2_age_s));

  // Once both relevant markers are stale/false, physical errors resume.
  left2_radar.update(false, at(42.0));
  assert(
    !global_radar.isActive(at(42.1), 1.0, global_age_s) &&
    !left2_radar.isActive(at(42.1), 1.0, left2_age_s));

  std::cout << "dummy source monitor tests passed" << std::endl;
  return 0;
}
