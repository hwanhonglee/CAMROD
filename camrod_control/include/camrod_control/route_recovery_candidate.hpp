#pragma once

// HH_260803 / TODOLIST 12 - Select only an unambiguous, fully evaluated
// translation for the bounded route-safety recovery controller.

#include <algorithm>
#include <cmath>
#include <string>

#include "avg_msgs/msg/avg_twist.hpp"
#include "camrod_control/motion_cost_stop.hpp"

namespace camrod_control
{

enum class RouteRecoveryCandidateKind
{
  kNone,
  kCrabLeft,
  kCrabRight,
  kReverse
};

struct RouteRecoveryCandidate
{
  RouteRecoveryCandidateKind kind{RouteRecoveryCandidateKind::kNone};
  avg_msgs::msg::AvgTwist command;
  std::string reason{"none"};

  bool available() const {return kind != RouteRecoveryCandidateKind::kNone;}
};

inline std::string routeRecoveryCandidateName(const RouteRecoveryCandidateKind kind)
{
  switch (kind) {
    case RouteRecoveryCandidateKind::kCrabLeft:
      return "crab_left";
    case RouteRecoveryCandidateKind::kCrabRight:
      return "crab_right";
    case RouteRecoveryCandidateKind::kReverse:
      return "reverse";
    case RouteRecoveryCandidateKind::kNone:
    default:
      return "none";
  }
}

inline avg_msgs::msg::AvgTwist routeRecoveryDirection(
  const avg_msgs::msg::AvgTwist & trigger,
  const RouteRecoveryCandidateKind kind,
  const double speed_mps,
  const double minimum_translation_mps = 0.02)
{
  avg_msgs::msg::AvgTwist command;
  const double norm = std::hypot(trigger.linear.x, trigger.linear.y);
  if (norm <= std::max(0.0, minimum_translation_mps)) {
    return command;
  }
  const double ux = trigger.linear.x / norm;
  const double uy = trigger.linear.y / norm;
  const double speed = std::max(0.0, speed_mps);
  if (kind == RouteRecoveryCandidateKind::kReverse) {
    command.linear.x = -ux * speed;
    command.linear.y = -uy * speed;
  } else if (kind == RouteRecoveryCandidateKind::kCrabLeft) {
    command.linear.x = -uy * speed;
    command.linear.y = ux * speed;
  } else if (kind == RouteRecoveryCandidateKind::kCrabRight) {
    command.linear.x = uy * speed;
    command.linear.y = -ux * speed;
  }
  return command;
}

inline RouteRecoveryCandidate selectRouteRecoveryCandidate(
  const avg_msgs::msg::AvgTwist & trigger,
  const double speed_mps,
  const MotionCostStopDecision & left,
  const MotionCostStopDecision & right,
  const MotionCostStopDecision & reverse,
  const double minimum_translation_mps = 0.02)
{
  if (std::hypot(trigger.linear.x, trigger.linear.y) <=
    std::max(0.0, minimum_translation_mps))
  {
    RouteRecoveryCandidate none;
    none.reason = "trigger_has_no_translation";
    return none;
  }

  const bool left_clear = !left.blocked;
  const bool right_clear = !right.blocked;
  if (left_clear != right_clear) {
    const auto kind = left_clear ? RouteRecoveryCandidateKind::kCrabLeft :
      RouteRecoveryCandidateKind::kCrabRight;
    return {kind, routeRecoveryDirection(trigger, kind, speed_mps, minimum_translation_mps),
      "unique_lateral_clear"};
  }
  if (!left_clear && !right_clear && !reverse.blocked) {
    return {RouteRecoveryCandidateKind::kReverse,
      routeRecoveryDirection(
        trigger, RouteRecoveryCandidateKind::kReverse, speed_mps,
        minimum_translation_mps),
      "lateral_blocked_reverse_clear"};
  }
  RouteRecoveryCandidate none;
  none.reason = left_clear ?
    "ambiguous_lateral_clear" : "no_projected_candidate_clear";
  return none;
}

inline RouteRecoveryCandidate continueRouteRecoveryCandidate(
  const avg_msgs::msg::AvgTwist & trigger,
  const RouteRecoveryCandidateKind latched_kind,
  const double speed_mps,
  const MotionCostStopDecision & left,
  const MotionCostStopDecision & right,
  const MotionCostStopDecision & reverse,
  const double minimum_translation_mps = 0.02)
{
  if (latched_kind == RouteRecoveryCandidateKind::kNone) {
    return selectRouteRecoveryCandidate(
      trigger, speed_mps, left, right, reverse, minimum_translation_mps);
  }

  const MotionCostStopDecision * decision = nullptr;
  if (latched_kind == RouteRecoveryCandidateKind::kCrabLeft) {
    decision = &left;
  } else if (latched_kind == RouteRecoveryCandidateKind::kCrabRight) {
    decision = &right;
  } else if (latched_kind == RouteRecoveryCandidateKind::kReverse) {
    decision = &reverse;
  }
  if (decision == nullptr || decision->blocked) {
    RouteRecoveryCandidate none;
    none.reason = "latched_" + routeRecoveryCandidateName(latched_kind) + "_blocked";
    if (decision != nullptr && !decision->reason.empty()) {
      none.reason += ":" + decision->reason;
    }
    return none;
  }
  return {
    latched_kind,
    routeRecoveryDirection(
      trigger, latched_kind, speed_mps, minimum_translation_mps),
    "latched_candidate_still_clear"};
}

}  // namespace camrod_control
