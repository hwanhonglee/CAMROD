#pragma once

// Keep the campsite crab terminal-speed and timeout policy independent from
// ROS time, publishers, and the maneuver state machine so its boundaries can
// be verified deterministically.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace camrod_control {

enum class CampsiteCrabEntryAction {
  kApproach,
  kRoadsideComplete,
  kBeginCentering,
  kContinueCentering,
  kBeginRotation,
};

inline double campsiteAdoptedWaitReturnStartYaw(
    const double current_yaw, const bool roadside_service,
    const bool roadside_reverse_return_enabled) {
  if (!std::isfinite(current_yaw)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // origin/develop retains the current heading for every roadside adoption;
  // only a turnaround site reconstructs its pre-turn entry yaw. The optional
  // reverse-return policy changes the later exit path, never this adoption
  // boundary.
  (void)roadside_reverse_return_enabled;
  if (roadside_service) {
    return current_yaw;
  }
  return std::atan2(std::sin(current_yaw - M_PI),
                    std::cos(current_yaw - M_PI));
}

inline CampsiteCrabEntryAction selectCampsiteCrabEntryAction(
    const bool entry_reached, const bool roadside_service,
    const bool centering_enabled, const bool centering_active,
    const bool centering_complete) {
  // Once centering starts it exclusively owns translation until it either
  // completes or fails. A noisy one-sided entry threshold must never switch
  // the wheels back to the ordinary crab-approach command.
  if (centering_active) {
    return centering_complete ? CampsiteCrabEntryAction::kBeginRotation
                              : CampsiteCrabEntryAction::kContinueCentering;
  }
  if (!entry_reached) {
    return CampsiteCrabEntryAction::kApproach;
  }
  if (roadside_service) {
    return CampsiteCrabEntryAction::kRoadsideComplete;
  }
  return centering_enabled ? CampsiteCrabEntryAction::kBeginCentering
                           : CampsiteCrabEntryAction::kBeginRotation;
}

enum class CampsiteEntryAnchorCenteringAction {
  kInactive,
  kContinue,
  kComplete,
  kTimeout,
};

enum class CampsiteCrabPostNominalAlignmentAction {
  kBeginAnchorCentering,
  kBeginBodyYawAlignment,
  kBeginCrabEntry,
};

inline CampsiteCrabPostNominalAlignmentAction
selectCampsiteCrabPostNominalAlignmentAction(
    const bool anchor_centering_enabled, const bool anchor_centering_completed,
    const bool body_yaw_compensation_active) {
  if (anchor_centering_enabled && !anchor_centering_completed) {
    return CampsiteCrabPostNominalAlignmentAction::kBeginAnchorCentering;
  }
  if (body_yaw_compensation_active) {
    return CampsiteCrabPostNominalAlignmentAction::kBeginBodyYawAlignment;
  }
  return CampsiteCrabPostNominalAlignmentAction::kBeginCrabEntry;
}

enum class CampsiteCrabPostEntryAction {
  kRestoreNominalYaw,
  kBeginRotation,
};

inline CampsiteCrabPostEntryAction
selectCampsiteCrabPostEntryAction(const bool body_yaw_compensation_active) {
  return body_yaw_compensation_active
             ? CampsiteCrabPostEntryAction::kRestoreNominalYaw
             : CampsiteCrabPostEntryAction::kBeginRotation;
}

inline CampsiteEntryAnchorCenteringAction
selectCampsiteEntryAnchorCenteringAction(const bool centering_active,
                                         const bool completion_ready,
                                         const bool steady_timeout_expired) {
  if (!centering_active) {
    return CampsiteEntryAnchorCenteringAction::kInactive;
  }
  // At the exact timeout boundary, fail closed even if the latest pose also
  // happens to enter tolerance. This keeps wall-clock execution bounded.
  if (steady_timeout_expired) {
    return CampsiteEntryAnchorCenteringAction::kTimeout;
  }
  return completion_ready ? CampsiteEntryAnchorCenteringAction::kComplete
                          : CampsiteEntryAnchorCenteringAction::kContinue;
}

struct CampsiteCrabEntrySafetyResult {
  double heading_drift_deg{0.0};
  double cross_track_error_m{0.0};
  bool invalid_input{false};
  bool heading_drift_exceeded{false};
  bool cross_track_exceeded{false};
};

inline bool campsiteCrabEntryBodyYawCompensationEligible(
    const double configured_compensation_deg,
    const bool automatic_lanelet_snap_entry, const bool turnaround_service) {
  return std::isfinite(configured_compensation_deg) &&
         configured_compensation_deg > 0.0 && automatic_lanelet_snap_entry &&
         turnaround_service;
}

inline double
campsiteCrabEntryBodyYawTarget(const double nominal_entry_yaw_rad,
                               const double crab_direction,
                               const double body_yaw_compensation_deg) {
  // Zero is an exact identity so the shared/physical controller keeps its
  // established heading and even avoids consulting the crab direction.
  if (body_yaw_compensation_deg == 0.0) {
    return nominal_entry_yaw_rad;
  }
  if (!std::isfinite(nominal_entry_yaw_rad) || !std::isfinite(crab_direction) ||
      crab_direction == 0.0 || !std::isfinite(body_yaw_compensation_deg) ||
      body_yaw_compensation_deg < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  constexpr double kDegreesToRadians = 0.017453292519943295769236907684886;
  const double signed_compensation_rad = std::copysign(
      body_yaw_compensation_deg * kDegreesToRadians, crab_direction);
  const double target = nominal_entry_yaw_rad + signed_compensation_rad;
  return std::atan2(std::sin(target), std::cos(target));
}

inline double
campsiteDirectedYawTargetDelta(const double rotation_start_yaw_rad,
                               const double target_yaw_rad,
                               const double rotation_direction) {
  if (!std::isfinite(rotation_start_yaw_rad) ||
      !std::isfinite(target_yaw_rad) || !std::isfinite(rotation_direction) ||
      rotation_direction == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double delta = std::atan2(std::sin(target_yaw_rad - rotation_start_yaw_rad),
                            std::cos(target_yaw_rad - rotation_start_yaw_rad));
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  if (rotation_direction > 0.0 && delta < 0.0) {
    delta += kTwoPi;
  } else if (rotation_direction < 0.0 && delta > 0.0) {
    delta -= kTwoPi;
  }
  return delta;
}

inline double campsiteIncrementalYawStep(const double previous_yaw_rad,
                                         const double current_yaw_rad) {
  if (!std::isfinite(previous_yaw_rad) || !std::isfinite(current_yaw_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(std::sin(current_yaw_rad - previous_yaw_rad),
                    std::cos(current_yaw_rad - previous_yaw_rad));
}

inline bool campsiteBodyYawWithinTolerance(const double current_yaw_rad,
                                           const double target_yaw_rad,
                                           const double tolerance_deg) {
  if (!std::isfinite(current_yaw_rad) || !std::isfinite(target_yaw_rad) ||
      !std::isfinite(tolerance_deg)) {
    return false;
  }
  constexpr double kRadiansToDegrees = 57.295779513082320876798154814105;
  const double error = std::atan2(std::sin(target_yaw_rad - current_yaw_rad),
                                  std::cos(target_yaw_rad - current_yaw_rad));
  return std::abs(error) * kRadiansToDegrees <= std::abs(tolerance_deg);
}

inline double campsiteCrabEntryLineCrossTrack(const double current_x,
                                              const double current_y,
                                              const double line_origin_x,
                                              const double line_origin_y,
                                              const double entry_yaw_rad) {
  const double dx = current_x - line_origin_x;
  const double dy = current_y - line_origin_y;
  // The intended CRAB_IN line is the body-lateral axis at entry_yaw. Its
  // signed cross-track error is therefore the fixed body-forward projection.
  return std::cos(entry_yaw_rad) * dx + std::sin(entry_yaw_rad) * dy;
}

inline CampsiteCrabEntrySafetyResult checkCampsiteCrabEntrySafety(
    const double current_yaw_rad, const double start_yaw_rad,
    const double cross_track_error_m, const double max_heading_drift_deg,
    const double max_cross_track_error_m) {
  CampsiteCrabEntrySafetyResult result;
  const bool heading_enabled = max_heading_drift_deg > 0.0;
  const bool cross_track_enabled = max_cross_track_error_m > 0.0;
  if (!heading_enabled && !cross_track_enabled) {
    return result;
  }
  if ((heading_enabled &&
       (!std::isfinite(current_yaw_rad) || !std::isfinite(start_yaw_rad) ||
        !std::isfinite(max_heading_drift_deg))) ||
      (cross_track_enabled && (!std::isfinite(cross_track_error_m) ||
                               !std::isfinite(max_cross_track_error_m)))) {
    result.invalid_input = true;
    return result;
  }
  if (heading_enabled) {
    constexpr double kRadiansToDegrees = 57.295779513082320876798154814105;
    const double delta = current_yaw_rad - start_yaw_rad;
    result.heading_drift_deg =
        std::abs(std::atan2(std::sin(delta), std::cos(delta))) *
        kRadiansToDegrees;
    result.heading_drift_exceeded =
        result.heading_drift_deg > max_heading_drift_deg;
  }
  if (cross_track_enabled) {
    result.cross_track_error_m = std::abs(cross_track_error_m);
    result.cross_track_exceeded =
        result.cross_track_error_m > max_cross_track_error_m;
  }
  return result;
}

inline double campsiteCrabApproachSpeed(
    const double max_speed_mps, const double min_speed_mps,
    const double slowdown_distance_m, const double remaining_to_completion_m) {
  const double maximum = std::max(0.0, max_speed_mps);
  const double minimum = std::clamp(min_speed_mps, 0.0, maximum);
  const double window = std::max(0.0, slowdown_distance_m);

  // A zero window is the disabled profile and must be an exact identity for
  // the configured maximum speed. The state machine owns the completion stop.
  if (window <= 1.0e-9 || remaining_to_completion_m >= window) {
    return maximum;
  }

  const double ratio = std::clamp(remaining_to_completion_m / window, 0.0, 1.0);
  return minimum + (maximum - minimum) * ratio;
}

inline double campsiteCrabApproachDuration(const double travel_distance_m,
                                           const double max_speed_mps,
                                           const double min_speed_mps,
                                           const double slowdown_distance_m) {
  const double invalid = std::numeric_limits<double>::infinity();
  if (!std::isfinite(travel_distance_m) || !std::isfinite(max_speed_mps) ||
      !std::isfinite(min_speed_mps) || !std::isfinite(slowdown_distance_m) ||
      travel_distance_m < 0.0 || max_speed_mps <= 0.0 || min_speed_mps < 0.0 ||
      slowdown_distance_m < 0.0) {
    return invalid;
  }
  if (travel_distance_m == 0.0) {
    return 0.0;
  }

  const double maximum = max_speed_mps;
  const double minimum = std::min(min_speed_mps, maximum);
  const double window = slowdown_distance_m;
  if (window <= 1.0e-9 || minimum >= maximum) {
    return travel_distance_m / maximum;
  }

  const double ramp_distance = std::min(travel_distance_m, window);
  if (minimum <= 0.0 && ramp_distance > 0.0) {
    // A linear profile ending at zero has an infinite time integral at the
    // completion boundary. Reject it instead of silently shortening timeout.
    return invalid;
  }

  const double cruise_distance = std::max(0.0, travel_distance_m - window);
  const double speed_at_ramp_start =
      minimum + (maximum - minimum) * ramp_distance / window;
  const double ramp_duration =
      window / (maximum - minimum) * std::log(speed_at_ramp_start / minimum);
  return cruise_distance / maximum + ramp_duration;
}

inline bool campsiteCrabSteadyTimeoutExpired(
    const std::chrono::steady_clock::time_point start,
    const std::chrono::steady_clock::time_point current,
    const std::chrono::steady_clock::duration timeout) {
  if (current < start) {
    return false;
  }
  return current - start >=
         std::max(timeout, std::chrono::steady_clock::duration::zero());
}

} // namespace camrod_control
