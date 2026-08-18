#pragma once

// HH_260818 - Separate one projected escape attempt from the complete recovery
// episode. This allows a fresh crab/reverse/yaw candidate to be retried without
// removing the independent total time and travel safety envelope.

#include <algorithm>

namespace camrod_control
{

enum class BoundedRecoveryAction
{
  kContinue,
  kRetry,
  kFinalHold,
};

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
