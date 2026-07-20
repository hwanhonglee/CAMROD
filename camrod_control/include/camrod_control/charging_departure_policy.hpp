#pragma once

// HH_260721 - Isolate bounded charger-departure authorization from ROS callbacks.

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <set>
#include <string>
#include <utility>

namespace camrod_control
{

struct ChargingDepartureConfig
{
  bool allow_mission_departure_while_charging{true};
  double grace_s{15.0};
  double mission_request_dedup_s{1.0};
  std::set<std::string> mission_prefixes{"camping_site_"};
};

struct MissionRequestIdentity
{
  std::string mission_key;
  std::string source;
  int32_t stamp_sec{0};
  uint32_t stamp_nanosec{0};

  bool operator==(const MissionRequestIdentity & other) const
  {
    return mission_key == other.mission_key && source == other.source &&
           stamp_sec == other.stamp_sec && stamp_nanosec == other.stamp_nanosec;
  }
};

class ChargingDeparturePolicy
{
public:
  explicit ChargingDeparturePolicy(ChargingDepartureConfig config = {})
  : config_(std::move(config))
  {
  }

  void setConfig(ChargingDepartureConfig config)
  {
    config_ = std::move(config);
    config_.grace_s = std::max(0.0, config_.grace_s);
    config_.mission_request_dedup_s = std::max(0.0, config_.mission_request_dedup_s);
  }

  void setCharging(const bool charging)
  {
    charging_ = charging;
    if (!charging_) {
      override_until_sec_ = 0.0;
    }
  }

  bool charging() const
  {
    return charging_;
  }

  bool requestDeparture(MissionRequestIdentity request, const double now_sec)
  {
    // HH_260721 - Accept only a new campsite request while CAN reports active charging.
    request.mission_key = normalizeLabel(request.mission_key);
    if (!charging_ || !config_.allow_mission_departure_while_charging ||
      !matchesMissionPrefix(request.mission_key))
    {
      return false;
    }

    if (has_last_request_ && request == last_request_ &&
      now_sec - last_request_time_sec_ < config_.mission_request_dedup_s)
    {
      return false;
    }

    has_last_request_ = true;
    last_request_ = std::move(request);
    last_request_time_sec_ = now_sec;
    override_until_sec_ = now_sec + config_.grace_s;
    return true;
  }

  void cancelDeparture()
  {
    override_until_sec_ = 0.0;
  }

  bool overrideActive(const double now_sec) const
  {
    return config_.allow_mission_departure_while_charging && charging_ &&
           override_until_sec_ > now_sec;
  }

  double remainingSec(const double now_sec) const
  {
    return overrideActive(now_sec) ? std::max(0.0, override_until_sec_ - now_sec) : 0.0;
  }

  const ChargingDepartureConfig & config() const
  {
    return config_;
  }

  static std::string normalizeLabel(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(), [](const unsigned char character) {
        if (character == '-' || std::isspace(character)) {
          return '_';
        }
        return static_cast<char>(std::tolower(character));
      });
    const auto first = value.find_first_not_of('_');
    if (first == std::string::npos) {
      return {};
    }
    const auto last = value.find_last_not_of('_');
    return value.substr(first, last - first + 1);
  }

private:
  bool matchesMissionPrefix(const std::string & mission_key) const
  {
    if (mission_key.empty()) {
      return false;
    }
    for (const auto & raw_prefix : config_.mission_prefixes) {
      const std::string prefix = normalizeLabel(raw_prefix);
      if (!prefix.empty() && mission_key.rfind(prefix, 0) == 0) {
        return true;
      }
    }
    return false;
  }

  ChargingDepartureConfig config_;
  bool charging_{false};
  bool has_last_request_{false};
  MissionRequestIdentity last_request_;
  double last_request_time_sec_{-1.0e9};
  double override_until_sec_{0.0};
};

}  // namespace camrod_control
