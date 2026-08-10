#pragma once

// HH_260805 / TODOLIST 12 - Select only fully projected crab, reverse, or
// bounded reverse-yaw commands and permit staged escape from a narrow contact.

#include <algorithm>
#include <cmath>
#include <string>

#include "avg_msgs/msg/avg_twist.hpp"
#include "camrod_control/motion_cost_stop.hpp"

namespace camrod_control {

enum class RouteRecoveryCandidateKind {
  kNone,
  kCrabLeft,
  kCrabRight,
  kReverse,
  kReverseYawLeft,
  kReverseYawRight
};

struct RouteRecoveryCandidate {
  RouteRecoveryCandidateKind kind{RouteRecoveryCandidateKind::kNone};
  avg_msgs::msg::AvgTwist command;
  std::string reason{"none"};

  bool available() const { return kind != RouteRecoveryCandidateKind::kNone; }
};

inline std::string
routeRecoveryCandidateName(const RouteRecoveryCandidateKind kind) {
  switch (kind) {
  case RouteRecoveryCandidateKind::kCrabLeft:
    return "crab_left";
  case RouteRecoveryCandidateKind::kCrabRight:
    return "crab_right";
  case RouteRecoveryCandidateKind::kReverse:
    return "reverse";
  case RouteRecoveryCandidateKind::kReverseYawLeft:
    return "reverse_yaw_left";
  case RouteRecoveryCandidateKind::kReverseYawRight:
    return "reverse_yaw_right";
  case RouteRecoveryCandidateKind::kNone:
  default:
    return "none";
  }
}

inline avg_msgs::msg::AvgTwist
routeRecoveryDirection(const avg_msgs::msg::AvgTwist &trigger,
                       const RouteRecoveryCandidateKind kind,
                       const double speed_mps,
                       const double minimum_translation_mps = 0.02,
                       const double yaw_rate_radps = 0.0) {
  avg_msgs::msg::AvgTwist command;
  const double norm = std::hypot(trigger.linear.x, trigger.linear.y);
  // HH_260810 - A manually selected goal can reach its snapped endpoint with
  // translation zero and only final yaw remaining. Use robot-forward as the
  // local reference so crab/reverse candidates are still generated; every
  // candidate remains subject to the projected body, planning, and obstacle
  // checks before it can reach the platform.
  double ux = 1.0;
  double uy = 0.0;
  if (norm > std::max(0.0, minimum_translation_mps)) {
    ux = trigger.linear.x / norm;
    uy = trigger.linear.y / norm;
  }
  const double speed = std::max(0.0, speed_mps);
  if (kind == RouteRecoveryCandidateKind::kReverse ||
      kind == RouteRecoveryCandidateKind::kReverseYawLeft ||
      kind == RouteRecoveryCandidateKind::kReverseYawRight) {
    command.linear.x = -ux * speed;
    command.linear.y = -uy * speed;
    if (kind == RouteRecoveryCandidateKind::kReverseYawLeft) {
      command.angular.z = std::abs(yaw_rate_radps);
    } else if (kind == RouteRecoveryCandidateKind::kReverseYawRight) {
      command.angular.z = -std::abs(yaw_rate_radps);
    }
  } else if (kind == RouteRecoveryCandidateKind::kCrabLeft) {
    command.linear.x = -uy * speed;
    command.linear.y = ux * speed;
  } else if (kind == RouteRecoveryCandidateKind::kCrabRight) {
    command.linear.x = uy * speed;
    command.linear.y = -ux * speed;
  }
  return command;
}

struct RouteRecoveryCandidateDecisions {
  MotionCostStopDecision crab_left;
  MotionCostStopDecision crab_right;
  MotionCostStopDecision reverse;
  MotionCostStopDecision reverse_yaw_left;
  MotionCostStopDecision reverse_yaw_right;
};

inline RouteRecoveryCandidateKind
uniqueClearKind(const MotionCostStopDecision &left,
                const MotionCostStopDecision &right,
                const RouteRecoveryCandidateKind left_kind,
                const RouteRecoveryCandidateKind right_kind) {
  if (left.blocked == right.blocked) {
    return RouteRecoveryCandidateKind::kNone;
  }
  return left.blocked ? right_kind : left_kind;
}

inline RouteRecoveryCandidateKind
projectedReverseYawKind(const avg_msgs::msg::AvgTwist &trigger,
                        const RouteRecoveryCandidateDecisions &decisions) {
  const auto unique =
      uniqueClearKind(decisions.reverse_yaw_left, decisions.reverse_yaw_right,
                      RouteRecoveryCandidateKind::kReverseYawLeft,
                      RouteRecoveryCandidateKind::kReverseYawRight);
  if (unique != RouteRecoveryCandidateKind::kNone) {
    return unique;
  }
  if (decisions.reverse_yaw_left.blocked ||
      decisions.reverse_yaw_right.blocked) {
    return RouteRecoveryCandidateKind::kNone;
  }

  // HH_260805 - If both short arcs are projected clear, preserve the RPP turn
  // sign instead of reversing forever. A nearly straight trigger remains a
  // straight reverse because it supplies no defensible yaw preference.
  constexpr double kMinimumPreferredYawRateRadps = 1.0e-3;
  if (trigger.angular.z > kMinimumPreferredYawRateRadps) {
    return RouteRecoveryCandidateKind::kReverseYawLeft;
  }
  if (trigger.angular.z < -kMinimumPreferredYawRateRadps) {
    return RouteRecoveryCandidateKind::kReverseYawRight;
  }
  return RouteRecoveryCandidateKind::kNone;
}

inline const MotionCostStopDecision *
routeRecoveryDecision(const RouteRecoveryCandidateDecisions &decisions,
                      const RouteRecoveryCandidateKind kind) {
  switch (kind) {
  case RouteRecoveryCandidateKind::kCrabLeft:
    return &decisions.crab_left;
  case RouteRecoveryCandidateKind::kCrabRight:
    return &decisions.crab_right;
  case RouteRecoveryCandidateKind::kReverse:
    return &decisions.reverse;
  case RouteRecoveryCandidateKind::kReverseYawLeft:
    return &decisions.reverse_yaw_left;
  case RouteRecoveryCandidateKind::kReverseYawRight:
    return &decisions.reverse_yaw_right;
  case RouteRecoveryCandidateKind::kNone:
  default:
    return nullptr;
  }
}

inline RouteRecoveryCandidate
makeRouteRecoveryCandidate(const avg_msgs::msg::AvgTwist &trigger,
                           const RouteRecoveryCandidateKind kind,
                           const double speed_mps, const double yaw_rate_radps,
                           const double minimum_translation_mps,
                           const std::string &reason) {
  return {kind,
          routeRecoveryDirection(trigger, kind, speed_mps,
                                 minimum_translation_mps, yaw_rate_radps),
          reason};
}

inline RouteRecoveryCandidate continueAdaptiveRouteRecoveryCandidate(
    const avg_msgs::msg::AvgTwist &trigger,
    const RouteRecoveryCandidateKind active_kind, const double speed_mps,
    const double yaw_rate_radps,
    const RouteRecoveryCandidateDecisions &decisions,
    const double minimum_translation_mps = 0.02) {
  const auto lateral =
      uniqueClearKind(decisions.crab_left, decisions.crab_right,
                      RouteRecoveryCandidateKind::kCrabLeft,
                      RouteRecoveryCandidateKind::kCrabRight);
  const auto reverse_yaw = projectedReverseYawKind(trigger, decisions);

  // HH_260805 - A unique projected crab direction directly identifies the
  // side opposite the contact and takes priority after reverse created room.
  if (lateral != RouteRecoveryCandidateKind::kNone && lateral != active_kind) {
    return makeRouteRecoveryCandidate(
        trigger, lateral, speed_mps, yaw_rate_radps, minimum_translation_mps,
        active_kind == RouteRecoveryCandidateKind::kNone
            ? "unique_lateral_clear"
            : "stage_switch_to_unique_lateral");
  }

  if (active_kind != RouteRecoveryCandidateKind::kNone) {
    const auto *active_decision = routeRecoveryDecision(decisions, active_kind);
    if (active_decision != nullptr && !active_decision->blocked) {
      // Once reversing, a uniquely safe yaw direction is useful before the
      // same straight retry can enter the same boundary contact again.
      if (active_kind == RouteRecoveryCandidateKind::kReverse &&
          reverse_yaw != RouteRecoveryCandidateKind::kNone) {
        return makeRouteRecoveryCandidate(
            trigger, reverse_yaw, speed_mps, yaw_rate_radps,
            minimum_translation_mps, "stage_switch_to_unique_reverse_yaw");
      }
      return makeRouteRecoveryCandidate(trigger, active_kind, speed_mps,
                                        yaw_rate_radps, minimum_translation_mps,
                                        "active_stage_still_clear");
    }
  }

  if (!decisions.reverse.blocked) {
    return makeRouteRecoveryCandidate(
        trigger, RouteRecoveryCandidateKind::kReverse, speed_mps,
        yaw_rate_radps, minimum_translation_mps,
        active_kind == RouteRecoveryCandidateKind::kNone
            ? "lateral_blocked_reverse_clear"
            : "active_stage_blocked_reverse_reposition");
  }
  if (reverse_yaw != RouteRecoveryCandidateKind::kNone) {
    return makeRouteRecoveryCandidate(trigger, reverse_yaw, speed_mps,
                                      yaw_rate_radps, minimum_translation_mps,
                                      "unique_reverse_yaw_clear");
  }

  RouteRecoveryCandidate none;
  none.reason = active_kind == RouteRecoveryCandidateKind::kNone
                    ? "no_projected_candidate_clear"
                    : "active_" + routeRecoveryCandidateName(active_kind) +
                          "_blocked_no_safe_transition";
  return none;
}

} // namespace camrod_control
