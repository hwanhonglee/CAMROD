#pragma once

// HH_260818 - Separate one projected escape attempt from the complete recovery
// episode. This allows a fresh crab/reverse/yaw candidate to be retried without
// removing the independent total time and travel safety envelope.

#include <algorithm>
#include <cmath>

#include "avg_msgs/msg/avg_twist.hpp"

namespace camrod_control
{

enum class BoundedRecoveryAction
{
  kContinue,
  kRetry,
  kFinalHold,
};

// The tuned route-relative policy uses an exact zero Twist as a fail-closed
// hold sentinel when no currently projected motion is safe. The shared develop
// behavior still treats it as an out-of-bounds candidate unless the owning node
// explicitly enables zero-hold timing. Nonzero sub-threshold commands remain
// invalid in either profile.
enum class BoundedRecoveryCandidateDisposition
{
  kMotion,
  kZeroHold,
  kInvalid,
};

struct BoundedRecoveryBehaviorConfig
{
  // Both switches are false to preserve the origin/develop controller policy.
  // The CARLA tuned profile may opt into either behavior independently.
  bool zero_hold_pauses_limits{false};
  bool allow_corrective_yaw_beyond_limit{false};
};

// Preserve the whole-attempt yaw envelope without preventing a command that
// drives an already out-of-envelope offset back toward the attempt's start
// heading.  Crab motion can rotate the physical chassis on uneven terrain;
// clamping solely on abs(offset) would then delete the corrective reverse-yaw
// component and leave a straight reverse command that the safety gate rejects.
inline bool BoundedRecoveryYawCommandPermitted(
  const double signed_yaw_offset_rad,
  const double requested_yaw_rate_radps,
  const double maximum_absolute_yaw_offset_rad,
  const double epsilon = 1.0e-9)
{
  if (!std::isfinite(signed_yaw_offset_rad) ||
    !std::isfinite(requested_yaw_rate_radps) ||
    !std::isfinite(maximum_absolute_yaw_offset_rad))
  {
    return false;
  }
  const double maximum = std::max(0.0, maximum_absolute_yaw_offset_rad);
  if (std::abs(signed_yaw_offset_rad) < maximum) {
    return true;
  }
  const double deadband = std::max(0.0, epsilon);
  return requested_yaw_rate_radps * signed_yaw_offset_rad < -deadband;
}

inline bool BoundedRecoveryYawCommandPermittedForConfig(
  const double signed_yaw_offset_rad,
  const double requested_yaw_rate_radps,
  const double maximum_absolute_yaw_offset_rad,
  const BoundedRecoveryBehaviorConfig & config)
{
  if (config.allow_corrective_yaw_beyond_limit) {
    return BoundedRecoveryYawCommandPermitted(
      signed_yaw_offset_rad, requested_yaw_rate_radps,
      maximum_absolute_yaw_offset_rad);
  }
  // Match origin/develop's exact comparison (`yaw >= limit`) when the tuned
  // corrective policy is disabled.  That legacy comparison also defines its
  // NaN edge behavior; fail-closed finite validation belongs only to the
  // explicit simulator policy above.
  (void)requested_yaw_rate_radps;
  return !(std::abs(signed_yaw_offset_rad) >=
         maximum_absolute_yaw_offset_rad);
}

inline bool BoundedRecoveryCandidatePausesLimits(
  const BoundedRecoveryCandidateDisposition disposition,
  const BoundedRecoveryBehaviorConfig & config)
{
  return config.zero_hold_pauses_limits &&
         disposition == BoundedRecoveryCandidateDisposition::kZeroHold;
}

// Optional tuned clock that counts only time during which a bounded motion
// candidate is eligible to run. The node never starts this clock under its
// default develop-compatible policy. A simulator profile can opt in when its
// gate intentionally uses exact zero as a route-relative hold sentinel.
class BoundedRecoveryZeroHoldClock
{
public:
  void reset()
  {
    active_ = false;
    pause_retry_timer_ = false;
    zero_hold_start_s_ = 0.0;
    episode_paused_s_ = 0.0;
    attempt_paused_s_ = 0.0;
    retry_paused_s_ = 0.0;
  }

  bool active() const {return active_;}

  void begin(const double now_s, const bool pause_retry_timer)
  {
    if (active_) {
      return;
    }
    active_ = true;
    pause_retry_timer_ = pause_retry_timer;
    zero_hold_start_s_ = std::isfinite(now_s) ? now_s : 0.0;
  }

  void resume(const double now_s)
  {
    if (!active_) {
      return;
    }
    const double paused = currentPauseDuration(now_s);
    episode_paused_s_ += paused;
    attempt_paused_s_ += paused;
    if (pause_retry_timer_) {
      retry_paused_s_ += paused;
    }
    active_ = false;
    pause_retry_timer_ = false;
  }

  void resetAttempt() {attempt_paused_s_ = 0.0;}
  void resetRetry() {retry_paused_s_ = 0.0;}

  double episodeElapsed(const double now_s, const double start_s) const
  {
    return activeElapsed(now_s, start_s, episode_paused_s_, true);
  }

  double attemptElapsed(const double now_s, const double start_s) const
  {
    return activeElapsed(now_s, start_s, attempt_paused_s_, true);
  }

  double retryElapsed(const double now_s, const double start_s) const
  {
    return activeElapsed(
      now_s, start_s, retry_paused_s_, pause_retry_timer_);
  }

  double zeroHoldElapsed(const double now_s) const
  {
    return active_ ? currentPauseDuration(now_s) : 0.0;
  }

private:
  double currentPauseDuration(const double now_s) const
  {
    if (!active_ || !std::isfinite(now_s) ||
      !std::isfinite(zero_hold_start_s_))
    {
      return 0.0;
    }
    return std::max(0.0, now_s - zero_hold_start_s_);
  }

  double activeElapsed(
    const double now_s, const double start_s, const double completed_pause_s,
    const bool include_current_pause) const
  {
    if (!std::isfinite(now_s) || !std::isfinite(start_s)) {
      return 0.0;
    }
    const double current_pause = include_current_pause ?
      currentPauseDuration(now_s) : 0.0;
    return std::max(
      0.0, now_s - start_s - std::max(0.0, completed_pause_s) - current_pause);
  }

  bool active_{false};
  bool pause_retry_timer_{false};
  double zero_hold_start_s_{0.0};
  double episode_paused_s_{0.0};
  double attempt_paused_s_{0.0};
  double retry_paused_s_{0.0};
};

inline BoundedRecoveryCandidateDisposition ClassifyBoundedRecoveryCandidate(
  const avg_msgs::msg::AvgTwist & candidate,
  const double minimum_translation_mps = 0.02,
  const double maximum_yaw_rate_radps = 0.15,
  const double zero_epsilon = 1.0e-9)
{
  if (!std::isfinite(candidate.linear.x) ||
    !std::isfinite(candidate.linear.y) ||
    !std::isfinite(candidate.angular.z))
  {
    return BoundedRecoveryCandidateDisposition::kInvalid;
  }
  const double translation = std::hypot(candidate.linear.x, candidate.linear.y);
  const double yaw_rate = std::abs(candidate.angular.z);
  const double zero = std::max(0.0, zero_epsilon);
  if (translation <= zero && yaw_rate <= zero) {
    return BoundedRecoveryCandidateDisposition::kZeroHold;
  }
  if (translation < std::max(0.0, minimum_translation_mps) ||
    yaw_rate > std::max(0.0, maximum_yaw_rate_radps))
  {
    return BoundedRecoveryCandidateDisposition::kInvalid;
  }
  return BoundedRecoveryCandidateDisposition::kMotion;
}

inline BoundedRecoveryCandidateDisposition
ClassifyBoundedRecoveryCandidateForConfig(
  const avg_msgs::msg::AvgTwist & candidate,
  const BoundedRecoveryBehaviorConfig & config,
  const double minimum_translation_mps = 0.02,
  const double maximum_yaw_rate_radps = 0.15,
  const double zero_epsilon = 1.0e-9)
{
  if (config.zero_hold_pauses_limits) {
    return ClassifyBoundedRecoveryCandidate(
      candidate, minimum_translation_mps, maximum_yaw_rate_radps,
      zero_epsilon);
  }

  // The develop controller used these two comparisons directly. Preserve
  // their exact default behavior (including non-finite IEEE comparisons) and
  // reserve the stricter zero/non-finite classifier for the opt-in zero-hold
  // policy that needs the additional disposition.
  const double translation = std::hypot(candidate.linear.x, candidate.linear.y);
  const double yaw_rate = std::abs(candidate.angular.z);
  if (translation < minimum_translation_mps ||
    yaw_rate > maximum_yaw_rate_radps)
  {
    return BoundedRecoveryCandidateDisposition::kInvalid;
  }
  return BoundedRecoveryCandidateDisposition::kMotion;
}

struct BoundedRecoveryAttemptLimits
{
  double attempt_duration_s{10.0};
  double attempt_distance_m{0.40};
  int maximum_attempts{50};
  double total_duration_s{90.0};
  double total_distance_m{1.50};
};

inline BoundedRecoveryAction EvaluateBoundedRecoveryAttempt(
  const BoundedRecoveryAttemptLimits & limits,
  const int current_attempt,
  const double attempt_elapsed_s,
  const double attempt_distance_m,
  const double total_elapsed_s,
  const double total_distance_m)
{
  if (total_elapsed_s >= std::max(0.0, limits.total_duration_s) ||
    total_distance_m >= std::max(0.0, limits.total_distance_m))
  {
    return BoundedRecoveryAction::kFinalHold;
  }
  if (attempt_elapsed_s < std::max(0.0, limits.attempt_duration_s) &&
    attempt_distance_m < std::max(0.0, limits.attempt_distance_m))
  {
    return BoundedRecoveryAction::kContinue;
  }
  return current_attempt < std::max(1, limits.maximum_attempts) ?
         BoundedRecoveryAction::kRetry : BoundedRecoveryAction::kFinalHold;
}

}  // namespace camrod_control
