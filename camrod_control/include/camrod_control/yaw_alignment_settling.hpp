#pragma once

// HH_260810 - Require measured yaw stability before a rotation hands command
// ownership to a translating maneuver.

#include <algorithm>
#include <cmath>
#include <optional>

#include "camrod_control/motion_geometry.hpp"

namespace camrod_control {

struct YawAlignmentSettlingConfig {
  double tolerance_deg{5.0};
  double hold_s{0.8};
  double maximum_yaw_rate_degps{3.0};
};

class YawAlignmentSettling {
public:
  explicit YawAlignmentSettling(YawAlignmentSettlingConfig config = {})
      : config_(config) {}

  void setConfig(const YawAlignmentSettlingConfig &config) {
    config_ = config;
    reset();
  }

  void reset() {
    previous_sample_time_sec_.reset();
    previous_yaw_rad_.reset();
    stable_since_sec_.reset();
    within_tolerance_ = false;
    settled_ = false;
    yaw_rate_degps_ = 0.0;
    stable_duration_s_ = 0.0;
  }

  bool observe(const double error_rad, const double yaw_rad,
               const double sample_time_sec) {
    if (!std::isfinite(error_rad) || !std::isfinite(yaw_rad) ||
        !std::isfinite(sample_time_sec)) {
      reset();
      return false;
    }

    if (previous_sample_time_sec_.has_value() &&
        sample_time_sec <= *previous_sample_time_sec_) {
      // A timer may evaluate the same pose more than once. Do not let a stale
      // localization sample satisfy a wall-clock dwell by itself.
      return settled_;
    }

    bool rate_is_known = false;
    if (previous_sample_time_sec_.has_value() &&
        previous_yaw_rad_.has_value()) {
      const double dt = sample_time_sec - *previous_sample_time_sec_;
      if (dt > 1.0e-6) {
        yaw_rate_degps_ =
            std::abs(normalizeAngle(yaw_rad - *previous_yaw_rad_)) * 180.0 /
            M_PI / dt;
        rate_is_known = true;
      }
    }
    previous_sample_time_sec_ = sample_time_sec;
    previous_yaw_rad_ = yaw_rad;

    within_tolerance_ =
        std::abs(error_rad * 180.0 / M_PI) <= std::abs(config_.tolerance_deg);
    const double rate_limit = std::abs(config_.maximum_yaw_rate_degps);
    const bool rate_is_stable =
        rate_limit <= 0.0 || (rate_is_known && yaw_rate_degps_ <= rate_limit);
    if (!within_tolerance_ || !rate_is_stable) {
      stable_since_sec_.reset();
      stable_duration_s_ = 0.0;
      settled_ = false;
      return false;
    }

    if (!stable_since_sec_.has_value()) {
      stable_since_sec_ = sample_time_sec;
    }
    stable_duration_s_ = std::max(0.0, sample_time_sec - *stable_since_sec_);
    settled_ = stable_duration_s_ >= std::max(0.0, config_.hold_s);
    return settled_;
  }

  bool withinTolerance() const { return within_tolerance_; }
  bool settled() const { return settled_; }
  double yawRateDegps() const { return yaw_rate_degps_; }
  double stableDurationS() const { return stable_duration_s_; }

private:
  YawAlignmentSettlingConfig config_;
  std::optional<double> previous_sample_time_sec_;
  std::optional<double> previous_yaw_rad_;
  std::optional<double> stable_since_sec_;
  bool within_tolerance_{false};
  bool settled_{false};
  double yaw_rate_degps_{0.0};
  double stable_duration_s_{0.0};
};

} // namespace camrod_control
