#pragma once

// HJ_261105 - Cap translation for a bounded window after an obstacle/cost
// stop releases. The window starts at the first moving command after the
// stop, so the resume voice wait cannot consume it. Linear x/y and angular z
// are scaled by one factor to keep the commanded curvature; pure rotation is
// not limited.

#include <algorithm>
#include <cmath>
#include <optional>

namespace camrod_control {

struct ResumeSpeedLimitConfig {
  bool enabled{true};
  double duration_s{4.0};
  double max_linear_mps{0.2};
};

struct PlanarVelocity {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

class ResumeSpeedLimit {
public:
  explicit ResumeSpeedLimit(ResumeSpeedLimitConfig config = {})
      : config_(config) {}

  void setConfig(const ResumeSpeedLimitConfig &config) { config_ = config; }
  const ResumeSpeedLimitConfig &config() const { return config_; }

  // Called on every tick that the cost stop holds the vehicle.
  void onStopHeld() {
    pending_ = true;
    window_start_sec_.reset();
  }

  bool active(const double now_sec) const {
    return window_start_sec_.has_value() &&
           now_sec - *window_start_sec_ < config_.duration_s;
  }

  // Returns the command to publish. Arms the window on the first translating
  // command after a held stop.
  PlanarVelocity apply(const PlanarVelocity &command, const double now_sec) {
    const double speed = std::hypot(command.linear_x, command.linear_y);
    if (!config_.enabled || config_.duration_s <= 0.0 ||
        !std::isfinite(config_.max_linear_mps) || config_.max_linear_mps < 0.0) {
      pending_ = false;
      window_start_sec_.reset();
      return command;
    }
    if (pending_ && speed > kMovingEpsilonMps) {
      pending_ = false;
      window_start_sec_ = now_sec;
    }
    if (!active(now_sec) || speed <= config_.max_linear_mps) {
      return command;
    }
    const double scale = config_.max_linear_mps / speed;
    return {command.linear_x * scale, command.linear_y * scale,
            command.angular_z * scale};
  }

private:
  static constexpr double kMovingEpsilonMps = 1e-3;
  ResumeSpeedLimitConfig config_;
  bool pending_{false};
  std::optional<double> window_start_sec_;
};

}  // namespace camrod_control
