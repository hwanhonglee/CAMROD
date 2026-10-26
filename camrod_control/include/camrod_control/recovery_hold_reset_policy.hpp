#pragma once

// HH_261024 - An administrator may reset a software recovery episode only
// after motion ownership has been revoked and fresh feedback proves standstill.
// This policy grants no motion authority and never changes an emergency stop.
#include <cmath>
#include <string>

namespace camrod_control {

struct RecoveryHoldResetContext {
  double now_sec{0.0};
  bool manual_engaged{false};
  bool mission_engaged{false};
  bool maneuver_active{false};
  bool estop_active{false};
  bool dr_timeout_active{false};
  bool platform_received{false};
  double platform_received_sec{0.0};
  unsigned int vehicle_state{0};
  unsigned int platform_error_code{0};
  bool motion_received{false};
  double motion_received_sec{0.0};
  double motion_stamp_sec{0.0};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double yaw_rate_rps{0.0};
};

inline std::string recoveryHoldResetBlockReason(
    const RecoveryHoldResetContext &context) {
  if (context.manual_engaged || context.mission_engaged)
    return "disengage_required";
  if (context.maneuver_active) return "maneuver_cancel_pending";
  if (context.estop_active) return "estop_active";
  if (context.dr_timeout_active) return "dr_timeout_active";
  const auto fresh = [&context](const double stamp) {
    return std::isfinite(context.now_sec) && std::isfinite(stamp) &&
           stamp > 0.0 && context.now_sec >= stamp &&
           context.now_sec - stamp <= 0.5;
  };
  if (!context.platform_received || !fresh(context.platform_received_sec))
    return "platform_status_missing_or_stale";
  if (context.vehicle_state != 0U || context.platform_error_code != 0U)
    return "platform_fault_active";
  if (!context.motion_received || !fresh(context.motion_received_sec) ||
      !fresh(context.motion_stamp_sec)) return "motion_feedback_missing_or_stale";
  if (!std::isfinite(context.vx_mps) || !std::isfinite(context.vy_mps) ||
      !std::isfinite(context.yaw_rate_rps)) return "invalid_motion_feedback";
  if (std::hypot(context.vx_mps, context.vy_mps) > 0.03 ||
      std::abs(context.yaw_rate_rps) > 0.03) return "robot_not_stationary";
  return "";
}

}  // namespace camrod_control
