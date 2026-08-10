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

namespace camrod_control {

struct RouteSafetyRecoveryConfig {
  bool enabled{true};
  double clear_required_s{1.5};
  // HH_260810 - A manual snapped goal can require many independently projected
  // margin recoveries before its final yaw fits. The budget never bypasses a
  // physical-body contact or any dynamic obstacle/interlock check.
  int max_automatic_releases{50};
  double rapid_recontact_window_s{5.0};
  // Reset the local retry episode only after the robot has moved forward past
  // the original contact, rather than merely oscillating around the same cell.
  double progress_reset_distance_m{0.75};
  bool allow_opposite_recovery_command{true};
  double opposite_direction_cosine_max{-0.5};
  double minimum_translation_mps{0.02};
  double opposite_recovery_probe_distance_m{0.25};
  double pose_max_age_s{0.5};
};

class RouteSafetyRecovery {
public:
  explicit RouteSafetyRecovery(RouteSafetyRecoveryConfig config = {})
      : config_(config) {}

  void setConfig(const RouteSafetyRecoveryConfig &config) {
    config_ = config;
    if (!config_.enabled) {
      reset();
    }
  }

  bool observeViolation(const MotionCostStopDecision &decision,
                        const avg_msgs::msg::AvgTwist &command,
                        const double now_sec) {
    if (!config_.enabled || !decision.blocked || !decision.lanelet_violation) {
      return false;
    }
    latest_decision_ = decision;
    latest_reason_ = decision.reason;
    if (active_) {
      // HH_260810 - Count retries per contact region. A recovery command that
      // genuinely carries the robot beyond the original contact can open a
      // new episode even while the current hold is still being evaluated.
      updateReleaseEpisodeProgress(decision, trigger_command_);
      clear_since_sec_.reset();
      return false;
    }
    active_ = true;
    release_budget_reset_by_progress_ = false;
    recovery_motion_observed_ = false;
    trigger_command_ = command;
    trigger_reason_ = decision.reason;
    activated_sec_ = now_sec;
    clear_since_sec_.reset();
    updateReleaseEpisodeProgress(decision, command);
    const double recontact_window =
        std::max(0.0, config_.rapid_recontact_window_s);
    // Pose-aware contacts reset only through signed forward progress. Keep the
    // time window as a fail-safe fallback for older decisions without a pose.
    if (!release_episode_origin_valid_ &&
        (!last_release_sec_.has_value() ||
         now_sec - *last_release_sec_ > recontact_window)) {
      automatic_releases_in_window_ = 0;
      last_release_sec_.reset();
      seedReleaseEpisode(decision, command);
    }
    automatic_release_blocked_ = automatic_releases_in_window_ >=
                                 std::max(0, config_.max_automatic_releases);
    return true;
  }

  bool updateProbe(const MotionCostStopDecision &decision,
                   const double now_sec) {
    if (!active_) {
      return false;
    }
    latest_decision_ = decision;
    if (decision.blocked) {
      latest_reason_ = decision.reason;
      clear_since_sec_.reset();
      return false;
    }
    // HH_260807 - Braking/coasting can move a simulated or physical platform
    // briefly clear after the stop command. Do not release Nav2 until a fully
    // validated opposite/crab recovery command has actually reached the gate.
    if (!recovery_motion_observed_) {
      clear_since_sec_.reset();
      return false;
    }
    if (!clear_since_sec_.has_value()) {
      clear_since_sec_ = now_sec;
    }
    if (now_sec - *clear_since_sec_ < std::max(0.0, config_.clear_required_s)) {
      return false;
    }
    // HH_260807 - A chain of rapid clear/recontact episodes may release Nav2
    // only up to the configured field budget. Once exhausted, projected-safe
    // inward escape remains available but same-direction Nav2 resume waits for
    // operator re-engage or a later episode outside the rolling window.
    if (automatic_release_blocked_) {
      return false;
    }
    last_release_sec_ = now_sec;
    ++automatic_releases_in_window_;
    clearActiveHold();
    return true;
  }

  bool permitsProjectedRecoveryCandidate(
      const avg_msgs::msg::AvgTwist &command) const {
    // HH_260807 - A rapid-recontact latch blocks only another same-direction
    // Nav2 release.  It must not deadlock the robot on the boundary: every
    // recovery candidate still passes the physical-body sweep, full planning
    // footprint endpoint, live obstacle and platform-interlock checks.
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

    const double cosine = (trigger_x * command_x + trigger_y * command_y) /
                          (trigger_norm * command_norm);
    if (cosine <=
        std::clamp(config_.opposite_direction_cosine_max, -1.0, 0.0)) {
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

  void reset() {
    clearActiveHold();
    automatic_releases_in_window_ = 0;
    last_release_sec_.reset();
    release_episode_origin_valid_ = false;
    release_episode_progress_m_ = 0.0;
    last_progress_reset_distance_m_ = 0.0;
    release_budget_reset_by_progress_ = false;
  }

  bool automaticReleaseBlocked() const {
    return active_ && automatic_release_blocked_;
  }
  void observeRecoveryMotion() { recovery_motion_observed_ = active_; }
  bool recoveryMotionObserved() const { return recovery_motion_observed_; }
  int automaticReleasesInWindow() const {
    return automatic_releases_in_window_;
  }
  int maxAutomaticReleases() const {
    return std::max(0, config_.max_automatic_releases);
  }
  double releaseEpisodeProgressM() const { return release_episode_progress_m_; }
  double lastProgressResetDistanceM() const {
    return last_progress_reset_distance_m_;
  }
  bool releaseBudgetResetByProgress() const {
    return release_budget_reset_by_progress_;
  }
  bool consumeReleaseBudgetResetByProgress() {
    const bool reset = release_budget_reset_by_progress_;
    release_budget_reset_by_progress_ = false;
    return reset;
  }
  bool active() const { return active_; }
  const avg_msgs::msg::AvgTwist &triggerCommand() const {
    return trigger_command_;
  }
  const std::string &triggerReason() const { return trigger_reason_; }
  const std::string &latestReason() const { return latest_reason_; }
  const MotionCostStopDecision &latestDecision() const {
    return latest_decision_;
  }
  double activatedSec() const { return activated_sec_; }
  double oppositeRecoveryProbeDistance() const {
    return std::max(0.05, config_.opposite_recovery_probe_distance_m);
  }
  double poseMaxAge() const { return std::max(0.0, config_.pose_max_age_s); }

  double clearElapsed(const double now_sec) const {
    return clear_since_sec_.has_value()
               ? std::max(0.0, now_sec - *clear_since_sec_)
               : 0.0;
  }

private:
  void seedReleaseEpisode(const MotionCostStopDecision &decision,
                          const avg_msgs::msg::AvgTwist &command) {
    if (!decision.lanelet_contact_valid) {
      return;
    }
    release_episode_origin_x_ = decision.lanelet_pose_x;
    release_episode_origin_y_ = decision.lanelet_pose_y;
    const double translation_norm =
        std::hypot(command.linear.x, command.linear.y);
    double body_direction_x = 1.0;
    double body_direction_y = 0.0;
    if (translation_norm > std::max(0.0, config_.minimum_translation_mps)) {
      body_direction_x = command.linear.x / translation_norm;
      body_direction_y = command.linear.y / translation_norm;
    }
    const double cosine = std::cos(decision.lanelet_pose_yaw);
    const double sine = std::sin(decision.lanelet_pose_yaw);
    release_episode_direction_x_ =
        cosine * body_direction_x - sine * body_direction_y;
    release_episode_direction_y_ =
        sine * body_direction_x + cosine * body_direction_y;
    release_episode_origin_valid_ = true;
    release_episode_progress_m_ = 0.0;
  }

  void updateReleaseEpisodeProgress(const MotionCostStopDecision &decision,
                                    const avg_msgs::msg::AvgTwist &command) {
    if (!decision.lanelet_contact_valid) {
      return;
    }
    if (!release_episode_origin_valid_) {
      seedReleaseEpisode(decision, command);
      return;
    }
    const double dx = decision.lanelet_pose_x - release_episode_origin_x_;
    const double dy = decision.lanelet_pose_y - release_episode_origin_y_;
    release_episode_progress_m_ =
        dx * release_episode_direction_x_ + dy * release_episode_direction_y_;
    if (release_episode_progress_m_ <
        std::max(0.05, config_.progress_reset_distance_m)) {
      return;
    }
    automatic_releases_in_window_ = 0;
    last_release_sec_.reset();
    automatic_release_blocked_ = false;
    release_budget_reset_by_progress_ = true;
    last_progress_reset_distance_m_ = release_episode_progress_m_;
    seedReleaseEpisode(decision, command);
  }

  void clearActiveHold() {
    active_ = false;
    trigger_command_ = avg_msgs::msg::AvgTwist{};
    trigger_reason_.clear();
    latest_reason_.clear();
    latest_decision_ = MotionCostStopDecision{};
    activated_sec_ = 0.0;
    clear_since_sec_.reset();
    automatic_release_blocked_ = false;
    recovery_motion_observed_ = false;
  }

  RouteSafetyRecoveryConfig config_;
  bool active_{false};
  bool automatic_release_blocked_{false};
  bool recovery_motion_observed_{false};
  int automatic_releases_in_window_{0};
  avg_msgs::msg::AvgTwist trigger_command_;
  std::string trigger_reason_;
  std::string latest_reason_;
  MotionCostStopDecision latest_decision_;
  double activated_sec_{0.0};
  std::optional<double> clear_since_sec_;
  std::optional<double> last_release_sec_;
  bool release_episode_origin_valid_{false};
  bool release_budget_reset_by_progress_{false};
  double release_episode_origin_x_{0.0};
  double release_episode_origin_y_{0.0};
  double release_episode_direction_x_{1.0};
  double release_episode_direction_y_{0.0};
  double release_episode_progress_m_{0.0};
  double last_progress_reset_distance_m_{0.0};
};

} // namespace camrod_control
