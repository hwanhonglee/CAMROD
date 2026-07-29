#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace camrod_sensing
{

// HH_260729 - A disabled SEN0592 channel retains its transport contract without
// creating an obstacle: every dummy range is strictly greater than max_range.
inline constexpr double kRadarNoTargetEpsilonM = 0.001;

inline double radar_no_target_range_m(const double max_range_m)
{
  if (!std::isfinite(max_range_m) || max_range_m <= 0.0) {
    throw std::invalid_argument("radar max_range must be finite and positive");
  }
  return max_range_m + kRadarNoTargetEpsilonM;
}

// HH_260729 - Treat dummy state as a positive, fresh heartbeat only. A stale
// marker must never suppress costs after the physical driver is restarted.
inline bool radar_dummy_state_suppresses_cost(
  const bool received,
  const bool active,
  const double age_s,
  const double timeout_s)
{
  return received && active &&
         std::isfinite(age_s) && age_s >= 0.0 &&
         std::isfinite(timeout_s) && timeout_s > 0.0 &&
         age_s <= timeout_s;
}

// Keep the dummy-state topic beside the channel's canonical AvgRange topic.
// This accepts both absolute driver defaults and namespace-relative YAML topics.
inline std::string radar_dummy_active_topic(const std::string & range_topic)
{
  constexpr char kRangeSuffix[] = "/range";
  constexpr std::size_t kRangeSuffixLength = sizeof(kRangeSuffix) - 1U;

  if (range_topic.size() <= kRangeSuffixLength ||
    range_topic.compare(
      range_topic.size() - kRangeSuffixLength,
      kRangeSuffixLength,
      kRangeSuffix) != 0)
  {
    throw std::invalid_argument(
            "radar range topic must end with /range: " + range_topic);
  }
  return range_topic.substr(0, range_topic.size() - kRangeSuffixLength) +
         "/dummy_active";
}

}  // namespace camrod_sensing
