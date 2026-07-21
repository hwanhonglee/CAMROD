#pragma once

// HH_260721 - Keep final command authorization deterministic and independently testable.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace camrod_control
{

struct CmdVelGatePolicyConfig
{
  bool require_platform_drive_enable{true};
  bool platform_safety_enabled{true};
  bool block_on_platform_status_stale{true};
  double platform_status_timeout_s{0.5};
  bool block_on_charging{true};
  bool block_on_platform_error_code{true};
  bool require_can_control_mode{true};
  bool critical_battery_stop_enabled{true};
  double critical_battery_percentage{0.10};
};

struct PlatformSafetyState
{
  bool received{false};
  double received_sec{0.0};
  uint8_t vehicle_state{0};
  uint8_t control_mode{0};
  uint16_t error_code{0};
  std::optional<double> battery_percentage;
};

class CmdVelGatePolicy
{
public:
  explicit CmdVelGatePolicy(CmdVelGatePolicyConfig config = {})
  : config_(config)
  {
  }

  void setConfig(const CmdVelGatePolicyConfig & config)
  {
    config_ = config;
  }

  void setManualEngage(const bool enabled)
  {
    manual_enabled_ = enabled;
  }

  void setMissionEngage(const bool enabled)
  {
    mission_enabled_ = enabled;
  }

  void setPlatformDriveEnable(const bool enabled)
  {
    platform_drive_enabled_ = enabled;
  }

  void setDrTimeout(const bool timed_out)
  {
    dr_timeout_ = timed_out;
  }

  void setEstopSource(const std::string & source, const bool active)
  {
    estop_sources_[source] = active;
  }

  void setPlatformState(const PlatformSafetyState & state)
  {
    platform_state_ = state;
  }

  void setCostState(const bool latched, const double hold_until_sec)
  {
    cost_latched_ = latched;
    cost_hold_until_sec_ = hold_until_sec;
  }

  void setGnssRecoveryHoldUntil(const double hold_until_sec)
  {
    gnss_recovery_hold_until_sec_ = hold_until_sec;
  }

  bool planningEngaged() const
  {
    return manual_enabled_ || mission_enabled_;
  }

  bool estopActive() const
  {
    return std::any_of(
      estop_sources_.begin(), estop_sources_.end(),
      [](const auto & source) {return source.second;});
  }

  std::vector<std::string> blockReasons(
    const double now_sec,
    const bool charging,
    const bool charging_motion_override) const
  {
    // HH_260721 - Return one authoritative reason list for admission, status, and logs.
    std::vector<std::string> reasons;
    if (!planningEngaged()) {
      reasons.push_back(
        "engage=false(manual=" + booleanText(manual_enabled_) +
        ",mission=" + booleanText(mission_enabled_) + ")");
    }
    if (estopActive()) {
      reasons.emplace_back("estop");
    }
    if (dr_timeout_) {
      reasons.emplace_back("dr_timeout");
    }
    if (config_.require_platform_drive_enable && !platform_drive_enabled_) {
      reasons.emplace_back("platform_drive_enable=false");
    }

    appendPlatformReasons(now_sec, charging, charging_motion_override, reasons);
    if (cost_latched_) {
      reasons.emplace_back("cost_stop_latched");
    } else if (cost_hold_until_sec_ > now_sec) {
      reasons.emplace_back("cost_hold=" + fixed(cost_hold_until_sec_ - now_sec, 1) + "s");
    }
    if (gnss_recovery_hold_until_sec_ > now_sec) {
      reasons.emplace_back(
        "gnss_recovery_hold=" + fixed(gnss_recovery_hold_until_sec_ - now_sec, 1) + "s");
    }
    return reasons;
  }

  bool enabled(
    const double now_sec,
    const bool charging,
    const bool charging_motion_override) const
  {
    return blockReasons(now_sec, charging, charging_motion_override).empty();
  }

  const PlatformSafetyState & platformState() const
  {
    return platform_state_;
  }

  bool manualEnabled() const {return manual_enabled_;}
  bool missionEnabled() const {return mission_enabled_;}
  bool platformDriveEnabled() const {return platform_drive_enabled_;}

  static std::string formatPlatformErrorCode(const uint16_t error_code)
  {
    static const std::vector<std::pair<uint16_t, std::string>> error_bits{
      {0x0001, "battery_fault"}, {0x0002, "battery_warning"},
      {0x0004, "remote_signal_lost"}, {0x0008, "motor1_communication"},
      {0x0010, "motor2_communication"}, {0x0020, "motor3_communication"},
      {0x0040, "motor4_communication"}, {0x0080, "steering_encoder"},
      {0x0100, "motor_driver"}, {0x0200, "upper_controller_communication"}};

    std::vector<std::string> labels;
    uint16_t known_mask = 0;
    for (const auto & entry : error_bits) {
      known_mask = static_cast<uint16_t>(known_mask | entry.first);
      if ((error_code & entry.first) != 0U) {
        labels.push_back(entry.second);
      }
    }
    if ((error_code & static_cast<uint16_t>(~known_mask)) != 0U) {
      labels.emplace_back("unknown_bits");
    }

    std::ostringstream output;
    output << "0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
           << error_code << "(";
    for (std::size_t index = 0; index < labels.size(); ++index) {
      if (index > 0U) {
        output << "+";
      }
      output << labels[index];
    }
    output << ")";
    return output.str();
  }

private:
  void appendPlatformReasons(
    const double now_sec,
    const bool charging,
    const bool charging_motion_override,
    std::vector<std::string> & reasons) const
  {
    if (!config_.platform_safety_enabled) {
      return;
    }
    if (config_.block_on_platform_status_stale) {
      if (!platform_state_.received) {
        reasons.emplace_back("platform_status_missing");
      } else {
        const double age_s = std::max(0.0, now_sec - platform_state_.received_sec);
        if (config_.platform_status_timeout_s > 0.0 &&
          age_s > config_.platform_status_timeout_s)
        {
          reasons.emplace_back("platform_status_stale=" + fixed(age_s, 2) + "s");
        }
      }
    }
    if (platform_state_.vehicle_state != 0U) {
      reasons.emplace_back("vehicle_state=" + std::to_string(platform_state_.vehicle_state));
    }
    if (config_.require_can_control_mode && platform_state_.control_mode != 1U) {
      reasons.emplace_back("control_mode=" + std::to_string(platform_state_.control_mode));
    }
    if (config_.block_on_platform_error_code && platform_state_.error_code != 0U) {
      reasons.emplace_back("platform_error=" + formatPlatformErrorCode(platform_state_.error_code));
    }
    if (config_.block_on_charging && charging && !charging_motion_override) {
      reasons.emplace_back("charging");
    }
    if (config_.critical_battery_stop_enabled && platform_state_.battery_percentage.has_value() &&
      std::isfinite(*platform_state_.battery_percentage) &&
      *platform_state_.battery_percentage <= config_.critical_battery_percentage)
    {
      reasons.emplace_back(
        "critical_battery=" + fixed(*platform_state_.battery_percentage * 100.0, 1) + "%");
    }
  }

  static std::string booleanText(const bool value)
  {
    return value ? "true" : "false";
  }

  static std::string fixed(const double value, const int precision)
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
  }

  CmdVelGatePolicyConfig config_;
  bool manual_enabled_{false};
  bool mission_enabled_{false};
  bool platform_drive_enabled_{false};
  bool dr_timeout_{false};
  std::map<std::string, bool> estop_sources_;
  PlatformSafetyState platform_state_;
  bool cost_latched_{false};
  double cost_hold_until_sec_{0.0};
  double gnss_recovery_hold_until_sec_{0.0};
};

}  // namespace camrod_control
