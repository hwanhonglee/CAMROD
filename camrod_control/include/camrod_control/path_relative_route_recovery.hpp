#pragma once

// Select a bounded route-recovery stage from the active full route.  This
// policy chooses direction only; MotionCostStop remains the authority that
// proves the complete physical-body sweep, planning footprint, dynamic costs,
// and platform interlocks before any command can be published.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

#include "avg_msgs/msg/avg_path.hpp"
#include "camrod_control/motion_cost_stop.hpp"
#include "camrod_control/route_recovery_candidate.hpp"

namespace camrod_control {

struct PathRelativeRecoveryConfig {
  // The shared CAMROD default retains the develop clearance-side recovery.
  // External-simulator profiles must opt in to this route-relative policy.
  bool enabled{false};
  double center_tolerance_m{0.05};
  double center_reentry_m{0.08};
  double heading_tolerance_rad{5.0 * 3.14159265358979323846 / 180.0};
  double path_max_age_s{0.5};
  double maximum_relation_distance_m{1.5};
};

struct PathRelativeRouteRelation {
  bool valid{false};
  // CAMROD path-tracking convention: positive is left of the ordered path.
  double signed_cte_m{0.0};
  // CAMROD path-tracking convention: robot yaw minus path tangent yaw.
  double heading_error_rad{0.0};
  double distance_m{0.0};
  double tangent_x{1.0};
  double tangent_y{0.0};
  std::size_t segment_index{0U};
  std::string reason{"path_relation_unavailable"};
};

struct PathRelativeRecoveryState {
  bool initialized{false};
  bool lateral_correction_active{false};
  bool settle_zero_commanded{false};
  bool projection_limited_zero_commanded{false};
  bool motion_authorized{false};
  double initial_absolute_cte_m{std::numeric_limits<double>::quiet_NaN()};
  double best_absolute_cte_m{std::numeric_limits<double>::infinity()};

  void reset() {
    initialized = false;
    lateral_correction_active = false;
    settle_zero_commanded = false;
    projection_limited_zero_commanded = false;
    motion_authorized = false;
    initial_absolute_cte_m = std::numeric_limits<double>::quiet_NaN();
    best_absolute_cte_m = std::numeric_limits<double>::infinity();
  }
};

inline double
pathRelativeAllowedCteRegression(const PathRelativeRecoveryConfig &config) {
  return std::max(0.0, config.center_reentry_m - config.center_tolerance_m);
}

inline bool
pathRelativeRecoveryHandoffReady(const PathRelativeRecoveryState &state,
                                 const PathRelativeRouteRelation &relation,
                                 const PathRelativeRecoveryConfig &config) {
  if (!state.initialized || !state.settle_zero_commanded || !relation.valid) {
    return false;
  }
  const double absolute_cte = std::abs(relation.signed_cte_m);
  const bool centered = !state.lateral_correction_active &&
                        absolute_cte < std::max(config.center_tolerance_m,
                                                config.center_reentry_m) &&
                        std::abs(relation.heading_error_rad) <=
                            std::max(0.0, config.heading_tolerance_rad);
  if (centered) {
    return true;
  }
  if (!state.projection_limited_zero_commanded ||
      !std::isfinite(state.initial_absolute_cte_m) ||
      !std::isfinite(state.best_absolute_cte_m)) {
    return false;
  }
  const double required_progress = std::max(0.0, config.center_tolerance_m);
  const double allowed_regression = pathRelativeAllowedCteRegression(config);
  return state.initial_absolute_cte_m - absolute_cte >= required_progress &&
         absolute_cte <= state.best_absolute_cte_m + allowed_regression &&
         std::abs(relation.heading_error_rad) <=
             std::max(0.0, config.heading_tolerance_rad);
}

inline double normalizePathRelativeAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline PathRelativeRouteRelation
nearestPathRelativeRouteRelation(const avg_msgs::msg::AvgPath &path,
                                 const PlanarPose &pose,
                                 const double maximum_relation_distance_m) {
  PathRelativeRouteRelation relation;
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw)) {
    relation.reason = "path_relation_pose_nonfinite";
    return relation;
  }
  if (!path.header.frame_id.empty() && !pose.frame_id.empty() &&
      path.header.frame_id != pose.frame_id) {
    relation.reason = "path_relation_frame_mismatch";
    return relation;
  }
  if (path.poses.size() < 2U) {
    relation.reason = "path_relation_insufficient_points";
    return relation;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < path.poses.size(); ++index) {
    const auto &a = path.poses[index].pose.position;
    const auto &b = path.poses[index + 1U].pose.position;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) ||
        !std::isfinite(b.y)) {
      continue;
    }
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double length_squared = vx * vx + vy * vy;
    if (!std::isfinite(length_squared) || length_squared <= 1.0e-9) {
      continue;
    }
    const double wx = pose.x - a.x;
    const double wy = pose.y - a.y;
    const double fraction =
        std::clamp((wx * vx + wy * vy) / length_squared, 0.0, 1.0);
    const double nearest_x = a.x + fraction * vx;
    const double nearest_y = a.y + fraction * vy;
    const double offset_x = pose.x - nearest_x;
    const double offset_y = pose.y - nearest_y;
    const double distance = std::hypot(offset_x, offset_y);
    if (!std::isfinite(distance) || distance >= best_distance) {
      continue;
    }
    const double cross = vx * (pose.y - a.y) - vy * (pose.x - a.x);
    const double length = std::sqrt(length_squared);
    // CTE is the signed normal projection, not the signed distance to the
    // clamped endpoint. When the local path has already pruned a vehicle that
    // is behind its first segment, the latter folds along-track gap into the
    // lateral error and can command a false multi-metre crab. Keep the clamped
    // Euclidean distance independently for the maximum-relation fail-close.
    relation.signed_cte_m = std::abs(cross) <= 1.0e-9 ? 0.0 : cross / length;
    relation.heading_error_rad =
        normalizePathRelativeAngle(pose.yaw - std::atan2(vy, vx));
    relation.distance_m = distance;
    relation.tangent_x = vx / length;
    relation.tangent_y = vy / length;
    relation.segment_index = index;
    best_distance = distance;
    relation.valid = true;
    relation.reason = "path_relation_valid";
  }

  if (!relation.valid) {
    relation.reason = "path_relation_no_finite_segment";
    return relation;
  }
  if (!std::isfinite(maximum_relation_distance_m) ||
      maximum_relation_distance_m <= 0.0 ||
      relation.distance_m > maximum_relation_distance_m) {
    relation.valid = false;
    relation.reason = "path_relation_too_far";
  }
  return relation;
}

inline double pathRelativeCteRate(const avg_msgs::msg::AvgTwist &command,
                                  const PlanarPose &pose,
                                  const PathRelativeRouteRelation &relation) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const double world_x = cosine * command.linear.x - sine * command.linear.y;
  const double world_y = sine * command.linear.x + cosine * command.linear.y;
  return relation.tangent_x * world_y - relation.tangent_y * world_x;
}

inline bool pathRelativeCommandDoesNotIncreaseAbsoluteCte(
    const avg_msgs::msg::AvgTwist &command, const PlanarPose &pose,
    const PathRelativeRouteRelation &relation,
    const double tolerance_mps = 1.0e-9) {
  if (!relation.valid || !std::isfinite(relation.signed_cte_m)) {
    return false;
  }
  const double cte_rate = pathRelativeCteRate(command, pose, relation);
  if (std::abs(relation.signed_cte_m) <= 1.0e-9) {
    return std::isfinite(cte_rate) &&
           std::abs(cte_rate) <= std::max(0.0, tolerance_mps);
  }
  const double cte_sign = relation.signed_cte_m > 0.0 ? 1.0 : -1.0;
  const double inward_rate = -cte_sign * cte_rate;
  return std::isfinite(inward_rate) &&
         inward_rate >= -std::max(0.0, tolerance_mps);
}

inline bool pathRelativeCommandStrictlyReducesAbsoluteCte(
    const avg_msgs::msg::AvgTwist &command, const PlanarPose &pose,
    const PathRelativeRouteRelation &relation,
    const double minimum_inward_rate_mps = 1.0e-9) {
  if (!relation.valid || !std::isfinite(relation.signed_cte_m) ||
      std::abs(relation.signed_cte_m) <= 1.0e-9) {
    return false;
  }
  const double cte_sign = relation.signed_cte_m > 0.0 ? 1.0 : -1.0;
  const double inward_rate =
      -cte_sign * pathRelativeCteRate(command, pose, relation);
  return std::isfinite(inward_rate) &&
         inward_rate > std::max(0.0, minimum_inward_rate_mps);
}

// The bounded recovery controller applies min(1, maximum_speed / norm) on
// every tick. Floating-point rotation can make a nominal 0.10 m/s diagonal
// have norm 0.10000000000000002, changing its last bits downstream and
// invalidating exact command identity. Move only an over-bound vector one ULP
// inside the same bound so the controller deterministically applies scale=1.
inline avg_msgs::msg::AvgTwist
pathRelativeControllerStableCommand(const avg_msgs::msg::AvgTwist &command,
                                    const double maximum_translation_mps) {
  avg_msgs::msg::AvgTwist stable = command;
  const double norm = std::hypot(stable.linear.x, stable.linear.y);
  if (!std::isfinite(norm) || !std::isfinite(maximum_translation_mps) ||
      maximum_translation_mps <= 0.0) {
    return avg_msgs::msg::AvgTwist{};
  }
  if (norm <= maximum_translation_mps) {
    return stable;
  }
  const double target = std::nextafter(maximum_translation_mps, 0.0);
  const double scale = target / norm;
  stable.linear.x *= scale;
  stable.linear.y *= scale;
  // Guard unusual libm rounding without ever increasing either component.
  for (int iteration = 0;
       iteration < 8 &&
       std::hypot(stable.linear.x, stable.linear.y) > maximum_translation_mps;
       ++iteration) {
    stable.linear.x = std::nextafter(stable.linear.x, 0.0);
    stable.linear.y = std::nextafter(stable.linear.y, 0.0);
  }
  if (std::hypot(stable.linear.x, stable.linear.y) > maximum_translation_mps) {
    return avg_msgs::msg::AvgTwist{};
  }
  return stable;
}

// Build the bounded lateral-recovery command from the ordered full-route
// normal itself.  The route relation is expressed in world coordinates, while
// AvgTwist is body-relative, so rotate the exact inward world normal back into
// the chassis frame.  Keeping both body x and y is important: replacing this
// vector with generic body-left/body-right crab can be almost tangent to a
// rotated route and can therefore increase CTE after CARLA steering limits are
// applied.
inline RouteRecoveryCandidate pathRelativeInwardNormalCrabCandidate(
    const PlanarPose &pose, const PathRelativeRouteRelation &relation,
    const double speed_mps, const double minimum_translation_mps = 0.02) {
  RouteRecoveryCandidate candidate;
  if (!relation.valid || !std::isfinite(relation.signed_cte_m) ||
      !std::isfinite(relation.tangent_x) ||
      !std::isfinite(relation.tangent_y) || !std::isfinite(pose.yaw)) {
    candidate.reason = "path_inward_normal_evidence_invalid";
    return candidate;
  }

  const double tangent_norm =
      std::hypot(relation.tangent_x, relation.tangent_y);
  if (!std::isfinite(tangent_norm) || tangent_norm <= 1.0e-9) {
    candidate.reason = "path_inward_normal_tangent_degenerate";
    return candidate;
  }
  if (!std::isfinite(speed_mps) || !std::isfinite(minimum_translation_mps) ||
      speed_mps <= 0.0 || speed_mps <= std::max(0.0, minimum_translation_mps)) {
    candidate.reason = "path_inward_normal_speed_below_minimum";
    return candidate;
  }
  if (std::abs(relation.signed_cte_m) <= 1.0e-9) {
    candidate.reason = "path_inward_normal_centered";
    return candidate;
  }

  const double tangent_x = relation.tangent_x / tangent_norm;
  const double tangent_y = relation.tangent_y / tangent_norm;
  const double cte_sign = relation.signed_cte_m > 0.0 ? 1.0 : -1.0;
  const double speed = std::max(0.0, speed_mps);

  // CTE rate is cross(tangent, world_velocity).  Therefore the unit vector
  // below produces d(CTE)/dt = -sign(CTE) * speed exactly.
  const double world_x = cte_sign * tangent_y * speed;
  const double world_y = -cte_sign * tangent_x * speed;
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  candidate.command.linear.x = cosine * world_x + sine * world_y;
  candidate.command.linear.y = -sine * world_x + cosine * world_y;
  candidate.command =
      pathRelativeControllerStableCommand(candidate.command, speed_mps);
  if (!std::isfinite(candidate.command.linear.x) ||
      !std::isfinite(candidate.command.linear.y)) {
    candidate.command = avg_msgs::msg::AvgTwist{};
    candidate.reason = "path_inward_normal_command_nonfinite";
    return candidate;
  }

  constexpr double kMinimumLateralComponentMps = 1.0e-9;
  if (candidate.command.linear.y > kMinimumLateralComponentMps) {
    candidate.kind = RouteRecoveryCandidateKind::kCrabLeft;
  } else if (candidate.command.linear.y < -kMinimumLateralComponentMps) {
    candidate.kind = RouteRecoveryCandidateKind::kCrabRight;
  } else {
    candidate.command = avg_msgs::msg::AvgTwist{};
    candidate.reason = "path_inward_normal_crab_side_unavailable";
    return candidate;
  }

  const double inward_rate =
      -cte_sign * pathRelativeCteRate(candidate.command, pose, relation);
  if (!std::isfinite(inward_rate) || inward_rate <= 1.0e-9) {
    candidate.kind = RouteRecoveryCandidateKind::kNone;
    candidate.command = avg_msgs::msg::AvgTwist{};
    candidate.reason = "path_inward_normal_not_inward";
    return candidate;
  }
  candidate.reason = "path_inward_full_route_normal_" +
                     routeRecoveryCandidateName(candidate.kind);
  return candidate;
}

// Build the heading-correction stage from current full-route geometry instead
// of assuming reverse is always CTE-safe. The yaw sign reduces heading error;
// body-forward and body-reverse translations are scored independently, with a
// strictly inward direction preferred and a zero-CTE-rate direction accepted
// only when neither direction can make strict progress.
inline RouteRecoveryCandidate pathRelativeHeadingCorrectionCandidate(
    const PlanarPose &pose, const PathRelativeRouteRelation &relation,
    const double speed_mps, const double yaw_rate_radps,
    const double minimum_translation_mps = 0.02) {
  RouteRecoveryCandidate unavailable;
  if (!relation.valid || !std::isfinite(relation.signed_cte_m) ||
      !std::isfinite(relation.heading_error_rad) ||
      !std::isfinite(relation.tangent_x) ||
      !std::isfinite(relation.tangent_y) || !std::isfinite(pose.yaw)) {
    unavailable.reason = "path_heading_candidate_evidence_invalid";
    return unavailable;
  }
  const double tangent_norm =
      std::hypot(relation.tangent_x, relation.tangent_y);
  if (!std::isfinite(tangent_norm) || tangent_norm <= 1.0e-9) {
    unavailable.reason = "path_heading_candidate_tangent_degenerate";
    return unavailable;
  }
  if (!std::isfinite(speed_mps) || !std::isfinite(minimum_translation_mps) ||
      speed_mps <= 0.0 || speed_mps <= std::max(0.0, minimum_translation_mps) ||
      !std::isfinite(yaw_rate_radps) || yaw_rate_radps <= 0.0) {
    unavailable.reason = "path_heading_candidate_speed_invalid";
    return unavailable;
  }
  if (std::abs(relation.heading_error_rad) <= 1.0e-9) {
    unavailable.reason = "path_heading_candidate_already_aligned";
    return unavailable;
  }

  const auto kind = relation.heading_error_rad > 0.0
                        ? RouteRecoveryCandidateKind::kReverseYawRight
                        : RouteRecoveryCandidateKind::kReverseYawLeft;
  const auto make_candidate = [&](const double body_x,
                                  const char *translation_name) {
    RouteRecoveryCandidate candidate;
    candidate.kind = kind;
    candidate.command.linear.x = body_x;
    candidate.command.angular.z = relation.heading_error_rad > 0.0
                                      ? -std::abs(yaw_rate_radps)
                                      : std::abs(yaw_rate_radps);
    candidate.command =
        pathRelativeControllerStableCommand(candidate.command, speed_mps);
    candidate.reason = "path_heading_" + std::string(translation_name) + "_" +
                       routeRecoveryCandidateName(kind);
    return candidate;
  };
  // Keep the full bounded translation. With deployed scale=0.5, raw
  // v/w=0.10/0.10 becomes final 0.05/0.05 and radius=1.0 m, above the Ranger
  // 0.810330349 m minimum Ackermann radius. MotionCostStop's projected Twist
  // arc therefore matches the adapter's executed steering semantics.
  const auto forward = make_candidate(std::abs(speed_mps), "forward");
  const auto reverse = make_candidate(-std::abs(speed_mps), "reverse");
  const bool forward_strict = pathRelativeCommandStrictlyReducesAbsoluteCte(
      forward.command, pose, relation);
  const bool reverse_strict = pathRelativeCommandStrictlyReducesAbsoluteCte(
      reverse.command, pose, relation);
  if (forward_strict) {
    return forward;
  }
  if (reverse_strict) {
    return reverse;
  }
  // Prefer reverse for the exact zero-rate tie to preserve the established
  // recovery convention, but never prefer it over an inward forward command.
  if (pathRelativeCommandDoesNotIncreaseAbsoluteCte(reverse.command, pose,
                                                    relation)) {
    return reverse;
  }
  if (pathRelativeCommandDoesNotIncreaseAbsoluteCte(forward.command, pose,
                                                    relation)) {
    return forward;
  }
  unavailable.reason = "path_heading_no_non_outward_translation";
  return unavailable;
}

inline bool
pathRelativeTwistExactlyMatches(const avg_msgs::msg::AvgTwist &lhs,
                                const avg_msgs::msg::AvgTwist &rhs) {
  return lhs.linear.x == rhs.linear.x && lhs.linear.y == rhs.linear.y &&
         lhs.linear.z == rhs.linear.z && lhs.angular.x == rhs.angular.x &&
         lhs.angular.y == rhs.angular.y && lhs.angular.z == rhs.angular.z;
}

inline bool
pathRelativeCandidateCommandExactlyMatches(const RouteRecoveryCandidate &lhs,
                                           const RouteRecoveryCandidate &rhs) {
  return lhs.kind == rhs.kind &&
         pathRelativeTwistExactlyMatches(lhs.command, rhs.command);
}

inline bool pathRelativeTwistIsFinite(const avg_msgs::msg::AvgTwist &command) {
  return std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
         std::isfinite(command.linear.z) && std::isfinite(command.angular.x) &&
         std::isfinite(command.angular.y) && std::isfinite(command.angular.z);
}

struct PathRelativeRecoveryCommandAuthorization {
  RouteRecoveryCandidateKind kind{RouteRecoveryCandidateKind::kNone};
  avg_msgs::msg::AvgTwist raw_command;
  avg_msgs::msg::AvgTwist evaluated_command;
  double authorized_sec{std::numeric_limits<double>::quiet_NaN()};
};

// Retain at most the current and immediately previous fully projected command.
// The previous slot closes the publisher/controller callback-order race, while
// exact six-field equality, stage identity and a short age bound prevent an
// arbitrary or old recovery command from inheriting that authorization.
struct PathRelativeRecoveryCommandLatch {
  std::optional<PathRelativeRecoveryCommandAuthorization> current;
  std::optional<PathRelativeRecoveryCommandAuthorization> previous;

  void reset() {
    current.reset();
    previous.reset();
  }

  bool authorize(const RouteRecoveryCandidate &candidate,
                 const avg_msgs::msg::AvgTwist &evaluated_command,
                 const double now_sec) {
    if (!candidate.available() || !std::isfinite(now_sec) || now_sec < 0.0 ||
        !pathRelativeTwistIsFinite(candidate.command) ||
        !pathRelativeTwistIsFinite(evaluated_command)) {
      reset();
      return false;
    }
    const double raw_norm =
        std::hypot(candidate.command.linear.x, candidate.command.linear.y);
    const double evaluated_norm =
        std::hypot(evaluated_command.linear.x, evaluated_command.linear.y);
    const double same_direction =
        candidate.command.linear.x * evaluated_command.linear.x +
        candidate.command.linear.y * evaluated_command.linear.y +
        candidate.command.linear.z * evaluated_command.linear.z +
        candidate.command.angular.x * evaluated_command.angular.x +
        candidate.command.angular.y * evaluated_command.angular.y +
        candidate.command.angular.z * evaluated_command.angular.z;
    if (!std::isfinite(raw_norm) || !std::isfinite(evaluated_norm) ||
        !std::isfinite(same_direction) || raw_norm <= 1.0e-9 ||
        evaluated_norm <= 1.0e-9 || evaluated_norm > raw_norm ||
        same_direction <= 0.0) {
      reset();
      return false;
    }

    PathRelativeRecoveryCommandAuthorization authorization{
        candidate.kind, candidate.command, evaluated_command, now_sec};
    if (current.has_value() && current->kind == authorization.kind &&
        pathRelativeTwistExactlyMatches(current->raw_command,
                                        authorization.raw_command) &&
        pathRelativeTwistExactlyMatches(current->evaluated_command,
                                        authorization.evaluated_command)) {
      current = authorization;
      return true;
    }
    previous = current;
    current = authorization;
    return true;
  }

  std::optional<PathRelativeRecoveryCommandAuthorization>
  authorizationForRawCommand(const avg_msgs::msg::AvgTwist &command,
                             const RouteRecoveryCandidateKind active_kind,
                             const double now_sec,
                             const double maximum_age_s) const {
    const auto matches = [&](const auto &authorization) {
      if (!authorization.has_value() || authorization->kind != active_kind ||
          !std::isfinite(now_sec) ||
          !std::isfinite(authorization->authorized_sec) ||
          !std::isfinite(maximum_age_s) || maximum_age_s < 0.0) {
        return false;
      }
      const double age_s = now_sec - authorization->authorized_sec;
      return age_s >= 0.0 && age_s <= maximum_age_s &&
             pathRelativeTwistExactlyMatches(command,
                                             authorization->raw_command);
    };
    if (matches(current)) {
      return current;
    }
    if (matches(previous)) {
      return previous;
    }
    return std::nullopt;
  }
};

inline RouteRecoveryCandidateKind pathRelativeInwardCrabKind(
    const avg_msgs::msg::AvgTwist &trigger, const PlanarPose &pose,
    const PathRelativeRouteRelation &relation, const double speed_mps,
    const double minimum_translation_mps = 0.02) {
  (void)trigger;
  return pathRelativeInwardNormalCrabCandidate(pose, relation, speed_mps,
                                               minimum_translation_mps)
      .kind;
}

// Project an inward crab only as far as the active route center. The selector
// is reevaluated continuously and stops lateral correction at the configured
// center tolerance; this distance is solely the complete-sweep proof horizon.
// A fixed 0.25 m horizon can cross the center and test the opposite boundary
// when only a few centimetres of CTE correction remain. Reverse and
// reverse-yaw candidates deliberately retain the configured full horizon.
inline double
pathRelativeInwardCrabProbeDistance(const avg_msgs::msg::AvgTwist &command,
                                    const RouteRecoveryCandidateKind kind,
                                    const PlanarPose &pose,
                                    const PathRelativeRouteRelation &relation,
                                    const double minimum_probe_distance_m,
                                    const double maximum_probe_distance_m) {
  const double minimum_probe = std::max(0.0, minimum_probe_distance_m);
  const double maximum_probe =
      std::max(minimum_probe, maximum_probe_distance_m);
  if (!relation.valid || (kind != RouteRecoveryCandidateKind::kCrabLeft &&
                          kind != RouteRecoveryCandidateKind::kCrabRight)) {
    return maximum_probe;
  }
  const double translation = std::hypot(command.linear.x, command.linear.y);
  if (!std::isfinite(translation) || translation <= 1.0e-9 ||
      !std::isfinite(relation.signed_cte_m)) {
    return maximum_probe;
  }
  const double cte_sign = relation.signed_cte_m >= 0.0 ? 1.0 : -1.0;
  const double inward_reduction_per_translation =
      -cte_sign * pathRelativeCteRate(command, pose, relation) / translation;
  if (!std::isfinite(inward_reduction_per_translation) ||
      inward_reduction_per_translation <= 1.0e-9) {
    return maximum_probe;
  }
  const double center_probe =
      std::abs(relation.signed_cte_m) / inward_reduction_per_translation;
  if (!std::isfinite(center_probe)) {
    return maximum_probe;
  }
  return std::clamp(center_probe, minimum_probe, maximum_probe);
}

inline RouteRecoveryCandidate selectPathRelativeRouteRecoveryCandidate(
    const avg_msgs::msg::AvgTwist &trigger, const PlanarPose &pose,
    const PathRelativeRouteRelation &relation,
    const PathRelativeRecoveryConfig &config, const double speed_mps,
    const double yaw_rate_radps,
    const RouteRecoveryCandidateDecisions &decisions,
    PathRelativeRecoveryState &state,
    const double minimum_translation_mps = 0.02,
    const RouteRecoveryCandidate *evaluated_inward_candidate = nullptr,
    const RouteRecoveryCandidate *evaluated_heading_candidate = nullptr) {
  (void)trigger;
  RouteRecoveryCandidate selected;
  state.settle_zero_commanded = false;
  state.projection_limited_zero_commanded = false;
  state.motion_authorized = false;
  if (!relation.valid) {
    selected.reason = relation.reason;
    return selected;
  }

  const double absolute_cte = std::abs(relation.signed_cte_m);
  if (!state.initialized) {
    state.initialized = true;
    state.initial_absolute_cte_m = absolute_cte;
    state.best_absolute_cte_m = absolute_cte;
    state.lateral_correction_active =
        absolute_cte > std::max(0.0, config.center_tolerance_m);
  } else if (state.lateral_correction_active &&
             absolute_cte <= std::max(0.0, config.center_tolerance_m)) {
    state.lateral_correction_active = false;
  } else if (!state.lateral_correction_active &&
             absolute_cte >=
                 std::max(config.center_tolerance_m, config.center_reentry_m)) {
    state.lateral_correction_active = true;
    state.initial_absolute_cte_m = absolute_cte;
    state.best_absolute_cte_m = absolute_cte;
  }

  // Lateral recovery is strictly first.  Heading correction while the chassis
  // is still outside the center band can trace an arc along the route edge and
  // recreate the same contact; only an inward crab may reduce that CTE.
  if (state.lateral_correction_active) {
    const double allowed_regression = pathRelativeAllowedCteRegression(config);
    if (std::isfinite(state.best_absolute_cte_m) &&
        absolute_cte > state.best_absolute_cte_m + allowed_regression) {
      selected.reason = "path_cte_regressed_beyond_hysteresis";
      return selected;
    }
    state.best_absolute_cte_m =
        std::min(state.best_absolute_cte_m, absolute_cte);
    const auto expected_inward_candidate =
        pathRelativeInwardNormalCrabCandidate(pose, relation, speed_mps,
                                              minimum_translation_mps);
    if (!expected_inward_candidate.available()) {
      selected.reason = expected_inward_candidate.reason;
      return selected;
    }
    const auto &inward_candidate = evaluated_inward_candidate == nullptr
                                       ? expected_inward_candidate
                                       : *evaluated_inward_candidate;
    if (!pathRelativeCandidateCommandExactlyMatches(
            inward_candidate, expected_inward_candidate)) {
      selected.reason = "path_inward_evaluated_command_mismatch";
      return selected;
    }
    const auto kind = inward_candidate.kind;
    const auto *projected = routeRecoveryDecision(decisions, kind);
    if (projected == nullptr || projected->blocked) {
      selected.reason =
          "path_inward_" + routeRecoveryCandidateName(kind) + "_blocked";
      if (projected != nullptr && !projected->reason.empty()) {
        selected.reason += ":" + projected->reason;
      }
      const bool planning_projection_limited =
          projected != nullptr && projected->blocked &&
          projected->lanelet_violation && !projected->dynamic_obstacle &&
          !projected->stale_grid &&
          projected->reason.rfind("route_recovery_predicted_lanelet_", 0U) ==
              0U;

      // A narrow route can reject the remaining inward-crab sweep even though
      // the correct-sign reverse-yaw sweep is completely clear.  At a large
      // heading error, holding zero here is a permanent lateral-first
      // deadlock: the chassis cannot change the geometry that blocked crab.
      // Permit only the heading-correcting reverse-yaw, and only when (1) the
      // crab rejection came exclusively from the planning lanelet projection,
      // (2) the alternative passed every projected safety source, and (3) its
      // translation cannot increase the current absolute route CTE.  Dynamic,
      // stale, physical-body, missing, wrong-sign, and outward alternatives
      // remain fail-closed.
      if (planning_projection_limited &&
          std::abs(relation.heading_error_rad) >
              std::max(0.0, config.heading_tolerance_rad)) {
        const auto expected_heading_candidate =
            pathRelativeHeadingCorrectionCandidate(pose, relation, speed_mps,
                                                   yaw_rate_radps,
                                                   minimum_translation_mps);
        if (!expected_heading_candidate.available()) {
          selected.reason += ":" + expected_heading_candidate.reason;
          return selected;
        }
        const auto &heading_candidate = evaluated_heading_candidate == nullptr
                                            ? expected_heading_candidate
                                            : *evaluated_heading_candidate;
        if (!pathRelativeCandidateCommandExactlyMatches(
                heading_candidate, expected_heading_candidate)) {
          selected.reason = "path_heading_evaluated_command_mismatch";
          return selected;
        }
        const auto *yaw_projection =
            routeRecoveryDecision(decisions, heading_candidate.kind);
        const bool yaw_projection_fully_clear =
            yaw_projection != nullptr && !yaw_projection->blocked &&
            !yaw_projection->dynamic_obstacle &&
            !yaw_projection->lanelet_violation && !yaw_projection->stale_grid;
        if (yaw_projection_fully_clear &&
            pathRelativeCommandDoesNotIncreaseAbsoluteCte(
                heading_candidate.command, pose, relation)) {
          selected = heading_candidate;
          selected.reason =
              "path_lateral_projection_limited_" + heading_candidate.reason;
          state.motion_authorized = true;
          return selected;
        }
      }
      const double verified_progress =
          state.initial_absolute_cte_m - state.best_absolute_cte_m;
      if (planning_projection_limited && std::isfinite(verified_progress) &&
          verified_progress >= std::max(0.0, config.center_tolerance_m)) {
        state.settle_zero_commanded = true;
        state.projection_limited_zero_commanded = true;
        selected.reason += ":projection_limited_zero_settle";
      }
      return selected;
    }
    // The evaluator-provided command is copied unchanged.  Never reconstruct
    // it from the enum here: that would discard the body-frame diagonal and
    // would make the projected sweep differ from the published command.
    selected = inward_candidate;
    selected.reason =
        "path_inward_full_route_normal_" + routeRecoveryCandidateName(kind);
    state.motion_authorized = true;
    return selected;
  }

  if (std::abs(relation.heading_error_rad) >
      std::max(0.0, config.heading_tolerance_rad)) {
    const auto expected_heading_candidate =
        pathRelativeHeadingCorrectionCandidate(
            pose, relation, speed_mps, yaw_rate_radps, minimum_translation_mps);
    if (!expected_heading_candidate.available()) {
      selected.reason = expected_heading_candidate.reason;
      return selected;
    }
    const auto &heading_candidate = evaluated_heading_candidate == nullptr
                                        ? expected_heading_candidate
                                        : *evaluated_heading_candidate;
    if (!pathRelativeCandidateCommandExactlyMatches(
            heading_candidate, expected_heading_candidate)) {
      selected.reason = "path_heading_evaluated_command_mismatch";
      return selected;
    }
    const auto *projected =
        routeRecoveryDecision(decisions, heading_candidate.kind);
    if (projected == nullptr || projected->blocked ||
        projected->dynamic_obstacle || projected->lanelet_violation ||
        projected->stale_grid) {
      selected.reason = "path_heading_inward_candidate_blocked";
      if (projected != nullptr && !projected->reason.empty()) {
        selected.reason += ":" + projected->reason;
      }
      return selected;
    }
    selected = heading_candidate;
    state.motion_authorized = true;
    return selected;
  }

  state.settle_zero_commanded = true;
  selected.reason = "path_centered_zero_settle";
  return selected;
}

} // namespace camrod_control
