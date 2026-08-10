#pragma once

#include <cstdint>

#include <avg_msgs/msg/avg_service_state.hpp>

namespace camrod_system::diagnostics
{

// HH_260810 - Separate expected service-owned Nav2 handoffs from real route
// failures. RETURNING_TO_DROP_ZONE is broad, so only its initial transition
// grace is suppressed; later return-route aborts remain operator-visible.
inline bool shouldSuppressNavAbort(
  int32_t service_state,
  double seconds_since_transition,
  double return_transition_grace_s,
  bool goal_existed_before_transition)
{
  switch (service_state) {
    case avg_msgs::msg::AvgServiceState::SITE_ENTRY:
    case avg_msgs::msg::AvgServiceState::UNLOAD_WAIT:
    case avg_msgs::msg::AvgServiceState::GUEST_LOADING_WAIT:
    case avg_msgs::msg::AvgServiceState::WAITING_FOR_RETURN_REQUEST:
    case avg_msgs::msg::AvgServiceState::RETURN_WITH_CARGO:
    case avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING:
    case avg_msgs::msg::AvgServiceState::WAITING_FOR_CHARGING:
    case avg_msgs::msg::AvgServiceState::OPERATOR_STOPPED:
      return true;
    case avg_msgs::msg::AvgServiceState::RETURNING_TO_DROP_ZONE:
      return goal_existed_before_transition &&
             return_transition_grace_s > 0.0 &&
             seconds_since_transition >= 0.0 &&
             seconds_since_transition <= return_transition_grace_s;
    default:
      return false;
  }
}

}  // namespace camrod_system::diagnostics
