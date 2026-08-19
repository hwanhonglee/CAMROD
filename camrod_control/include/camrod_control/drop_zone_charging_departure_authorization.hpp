#pragma once

// HH_260819 - Authorize charger departure from fresh controller progress,
// independently of planning mission identity.  This is intentionally a small
// ROS-free policy so exact phases, health and heartbeat expiry stay testable.

#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace camrod_control
{

struct DropZoneChargingDepartureAuthorizationConfig
{
  bool enabled{true};
  double heartbeat_timeout_s{2.0};
  double future_tolerance_s{0.25};
  bool require_source_stamp{true};
  std::string expected_module_name{"control"};
};

inline bool dropZoneChargingDepartureAuthorizationConfigIsValid(
  const DropZoneChargingDepartureAuthorizationConfig & config)
{
  return std::isfinite(config.heartbeat_timeout_s) &&
         config.heartbeat_timeout_s > 0.0 &&
         std::isfinite(config.future_tolerance_s) &&
         config.future_tolerance_s >= 0.0 &&
         config.future_tolerance_s <= config.heartbeat_timeout_s &&
         !config.expected_module_name.empty();
}

class DropZoneChargingDepartureAuthorization
{
public:
  explicit DropZoneChargingDepartureAuthorization(
    DropZoneChargingDepartureAuthorizationConfig config = {})
  : config_(std::move(config))
  {
  }

  void setConfig(DropZoneChargingDepartureAuthorizationConfig config)
  {
    config_ = std::move(config);
    if (!config_.enabled ||
        !dropZoneChargingDepartureAuthorizationConfigIsValid(config_)) {
      reset();
    }
  }

  static bool phaseAuthorizes(const std::string & phase)
  {
    // Exact, case-sensitive controller operating states are the contract.
    // Parking/idle/error text must never inherit this charging exception.
    return phase == "EXIT_STRAIGHT" || phase == "ALIGN_EXIT_YAW";
  }

  bool observe(
    const std::string & module_name, const std::string & phase,
    const bool healthy,
    const double received_sec,
    const std::optional<double> source_stamp_sec = std::nullopt)
  {
    if (!config_.enabled ||
        !dropZoneChargingDepartureAuthorizationConfigIsValid(config_) ||
        module_name != config_.expected_module_name ||
        !healthy || !phaseAuthorizes(phase) ||
        !std::isfinite(received_sec)) {
      reset();
      return false;
    }
    if (config_.require_source_stamp && !source_stamp_sec.has_value()) {
      reset();
      return false;
    }
    if (source_stamp_sec.has_value()) {
      if (!std::isfinite(*source_stamp_sec)) {
        reset();
        return false;
      }
      const double source_age_at_receipt = received_sec - *source_stamp_sec;
      if (source_age_at_receipt < -config_.future_tolerance_s ||
          source_age_at_receipt > config_.heartbeat_timeout_s) {
        reset();
        return false;
      }
    }

    phase_ = phase;
    received_sec_ = received_sec;
    source_stamp_sec_ = source_stamp_sec;
    has_valid_evidence_ = true;
    return true;
  }

  void reset()
  {
    phase_.clear();
    received_sec_ = 0.0;
    source_stamp_sec_.reset();
    has_valid_evidence_ = false;
  }

  bool isActive(const double now_sec) const
  {
    if (!config_.enabled || !has_valid_evidence_ ||
        !dropZoneChargingDepartureAuthorizationConfigIsValid(config_) ||
        !std::isfinite(now_sec) || !phaseAuthorizes(phase_)) {
      return false;
    }
    const double receipt_age = now_sec - received_sec_;
    if (receipt_age < -config_.future_tolerance_s ||
        receipt_age > config_.heartbeat_timeout_s) {
      return false;
    }
    if (source_stamp_sec_.has_value()) {
      const double source_age = now_sec - *source_stamp_sec_;
      if (source_age < -config_.future_tolerance_s ||
          source_age > config_.heartbeat_timeout_s) {
        return false;
      }
    }
    return true;
  }

  double receiptAgeSec(const double now_sec) const
  {
    return has_valid_evidence_ && std::isfinite(now_sec)
             ? now_sec - received_sec_
             : -1.0;
  }

  const std::string & phase() const
  {
    return phase_;
  }

  const DropZoneChargingDepartureAuthorizationConfig & config() const
  {
    return config_;
  }

private:
  DropZoneChargingDepartureAuthorizationConfig config_;
  std::string phase_;
  double received_sec_{0.0};
  std::optional<double> source_stamp_sec_;
  bool has_valid_evidence_{false};
};

}  // namespace camrod_control
