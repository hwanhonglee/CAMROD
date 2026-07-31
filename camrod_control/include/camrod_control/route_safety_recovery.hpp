#pragma once

// HH_260729 / TODOLIST 11-12 - Preserve the route-violation direction until
// the original lanelet probe is continuously clear, while allowing an
// explicitly opposite command to move the robot back toward a safe corridor.

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "avg_msgs/msg/avg_twist.hpp"
#include "camrod_control/motion_cost_stop.hpp"

namespace camrod_control
{

struct RouteSafetyRecoveryConfig
{
  bool enabled{true};
  double clear_required_s{1.0};
  bool allow_opposite_recovery_command{true};
  double opposite_direction_cosine_max{-0.5};
  double minimum_translation_mps{0.02};
  double opposite_recovery_probe_distance_m{0.25};
  double pose_max_age_s{0.5};
};

class RouteSafetyRecovery
{
public:
  explicit RouteSafetyRecovery(RouteSafetyRecoveryConfig config = {})
  : config_(config)
  {
  }

  void setConfig(const RouteSafetyRecoveryConfig & config)
  {
    config_ = config;
    if (!config_.enabled) {
      reset();
    }
  }

  bool observeViolation(
    const MotionCostStopDecision & decision,
    const avg_msgs::msg::AvgTwist & command,
    const double now_sec)
  {
    if (!config_.enabled || !decision.blocked || !decision.lanelet_violation) {
      return false;
    }
    latest_reason_ = decision.reason;
    if (active_) {
      clear_since_sec_.reset();
      return false;
    }
    active_ = true;
    trigger_command_ = command;
    trigger_reason_ = decision.reason;
    activated_sec_ = now_sec;
    clear_since_sec_.reset();
    return true;
  }

  bool updateProbe(const MotionCostStopDecision & decision, const double now_sec)
  {
    if (!active_) {
      return false;
    }
    if (decision.blocked) {
      latest_reason_ = decision.reason;
      clear_since_sec_.reset();
      return false;
    }
    if (!clear_since_sec_.has_value()) {
      clear_since_sec_ = now_sec;
    }
    if (now_sec - *clear_since_sec_ < std::max(0.0, config_.clear_required_s)) {
      return false;
    }
    reset();
    return true;
  }

  bool permitsProjectedRecoveryCandidate(const avg_msgs::msg::AvgTwist & command) const
  {
    if (!active_ || !config_.allow_opposite_recovery_command) {
      return false;
    }
    const double trigger_x = trigger_command_.linear.x;
    const double trigger_y = trigger_command_.linear.y;
    const double command_x = command.linear.x;
    const double command_y = command.linear.y;
    const double trigger_norm = std::hypot(trigger_x, trigger_y);
    const double command_norm = std::hypot(command_x, command_y);
    const double minimum = std::max(0.0, config_.minimum_translation_mps);
    if (command_norm <= minimum) {
      return false;
    }

    // HH_260731 - A rotation-only lanelet violation has no translation vector
    // to invert. Admit a translational candidate and let the projected complete
    // footprint + dynamic-cost evaluation below decide whether it moves clear.
    if (trigger_norm <= minimum) {
      return true;
    }

    const double cosine =
      (trigger_x * command_x + trigger_y * command_y) / (trigger_norm * command_norm);
    if (cosine <= std::clamp(config_.opposite_direction_cosine_max, -1.0, 0.0)) {
      return true;
    }

    // HH_260731 - A forward command can touch a side boundary even though the
    // safe escape is pure crab and therefore orthogonal to the trigger. Admit
    // only the orthogonal candidate here; evaluateRouteRecoveryCommand() must
    // still prove that the projected full footprint is clear. The candidate
    // toward the contacted boundary remains blocked by that projection.
    constexpr double kOrthogonalCosineTolerance = 1.0e-6;
    return std::abs(cosine) <= kOrthogonalCosineTolerance;
  }

  void reset()
  {
    active_ = false;
    trigger_command_ = avg_msgs::msg::AvgTwist{};
    trigger_reason_.clear();
    latest_reason_.clear();
    activated_sec_ = 0.0;
    clear_since_sec_.reset();
  }

  bool active() const {return active_;}
  const avg_msgs::msg::AvgTwist & triggerCommand() const {return trigger_command_;}
  const std::string & triggerReason() const {return trigger_reason_;}
  const std::string & latestReason() const {return latest_reason_;}
  double activatedSec() const {return activated_sec_;}
  double oppositeRecoveryProbeDistance() const
  {
    return std::max(0.05, config_.opposite_recovery_probe_distance_m);
  }
  double poseMaxAge() const {return std::max(0.0, config_.pose_max_age_s);}

  double clearElapsed(const double now_sec) const
  {
    return clear_since_sec_.has_value() ? std::max(0.0, now_sec - *clear_since_sec_) : 0.0;
  }

private:
  RouteSafetyRecoveryConfig config_;
  bool active_{false};
  avg_msgs::msg::AvgTwist trigger_command_;
  std::string trigger_reason_;
  std::string latest_reason_;
  double activated_sec_{0.0};
  std::optional<double> clear_since_sec_;
};

}  // namespace camrod_control
