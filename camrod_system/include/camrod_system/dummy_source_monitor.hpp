#pragma once

#include <cmath>
#include <mutex>

#include <rclcpp/time.hpp>

namespace camrod_system
{

// HH_260729 - A dummy stream may replace an intentionally disabled physical
// sensor during integration tests.  The heartbeat must remain fresh and true;
// a false or stale heartbeat immediately restores the normal fail-visible
// sensor diagnosis.
class DummySourceMonitor
{
public:
  void update(bool active, const rclcpp::Time & received_at)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = active;
    received_ = true;
    last_received_at_ = received_at;
  }

  bool isActive(
    const rclcpp::Time & now,
    double timeout_s,
    double & age_s) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || !active_ || !std::isfinite(timeout_s) || timeout_s <= 0.0) {
      age_s = -1.0;
      return false;
    }

    age_s = (now - last_received_at_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= timeout_s;
  }

private:
  mutable std::mutex mutex_;
  bool received_{false};
  bool active_{false};
  rclcpp::Time last_received_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace camrod_system
