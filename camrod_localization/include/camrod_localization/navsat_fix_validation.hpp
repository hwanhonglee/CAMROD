#pragma once

#include <cmath>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace camrod_localization
{

// HH_260729: Disabled-GNSS transport uses STATUS_NO_FIX + NaN coordinates.
// Keep the validation independent from the adapter callback so the safety
// contract can be unit-tested without launching the full localization graph.
inline bool navSatFixIsUsable(const sensor_msgs::msg::NavSatFix & msg)
{
  return
    msg.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    std::isfinite(msg.latitude) &&
    std::isfinite(msg.longitude) &&
    std::isfinite(msg.altitude) &&
    msg.latitude >= -90.0 && msg.latitude <= 90.0 &&
    msg.longitude >= -180.0 && msg.longitude <= 180.0;
}

}  // namespace camrod_localization
