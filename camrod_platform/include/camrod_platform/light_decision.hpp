// 260708 - Pure decision logic for exterior lights (no ROS dependency).
// Priority: HAZARD (estop) > crab direction > lanelet turn window > OFF.
// Kept header-only so gtest can exercise every branch deterministically,
// mirroring the planning cmd_vel gate test approach.
#ifndef CAMROD_PLATFORM__LIGHT_DECISION_HPP_
#define CAMROD_PLATFORM__LIGHT_DECISION_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace camrod_platform
{

enum class IndicatorMode : uint8_t
{
  kOff = 0,
  kLeft = 1,
  kRight = 2,
  kHazard = 3,
};

// One turn window on the route arc-length axis, as published by
// /planning/route_turn_segments: direction +1=left / -1=right.
struct TurnSegment
{
  float direction{0.0F};
  float start_s{0.0F};
  float end_s{0.0F};
};

struct LightDecisionInput
{
  bool estop_platform{false};
  bool estop_state_machine{false};
  bool site_maneuver_active{false};
  double cmd_vel_lateral_mps{0.0};
  // True only when pose/path are fresh and the route sync guard passed.
  bool route_context_valid{false};
  // Robot arc length along the current global path; negative = unknown.
  double route_s{-1.0};
  std::vector<TurnSegment> turn_segments;
};

struct LightDecisionParams
{
  double turn_pre_distance_m{12.0};
  double crab_lateral_threshold_mps{0.05};
};

struct LightDecisionResult
{
  IndicatorMode mode{IndicatorMode::kOff};
  std::string source{"off"};
};

// data[0] of route_turn_segments carries the planner-side total route length.
// A mismatch against the received global path means the pair is mid-swap
// (route change propagating) and turn windows must not be trusted this cycle.
inline bool routeLengthMatches(
  double path_length_m, double segments_total_m, double tolerance_m)
{
  return std::abs(path_length_m - segments_total_m) <= tolerance_m;
}

inline LightDecisionResult decideIndicator(
  const LightDecisionInput & input, const LightDecisionParams & params)
{
  if (input.estop_platform || input.estop_state_machine) {
    return {IndicatorMode::kHazard,
      input.estop_platform ? "hazard:estop_platform" : "hazard:estop_state_machine"};
  }

  if (input.site_maneuver_active &&
    std::abs(input.cmd_vel_lateral_mps) > params.crab_lateral_threshold_mps)
  {
    if (input.cmd_vel_lateral_mps > 0.0) {
      return {IndicatorMode::kLeft, "crab:left"};
    }
    return {IndicatorMode::kRight, "crab:right"};
  }

  if (input.route_context_valid && input.route_s >= 0.0) {
    const TurnSegment * active_segment = nullptr;
    for (const auto & segment : input.turn_segments) {
      const bool inside_window =
        input.route_s >= static_cast<double>(segment.start_s) - params.turn_pre_distance_m &&
        input.route_s <= static_cast<double>(segment.end_s);
      if (inside_window && (active_segment == nullptr ||
        segment.start_s < active_segment->start_s))
      {
        active_segment = &segment;
      }
    }
    if (active_segment != nullptr) {
      if (active_segment->direction > 0.0F) {
        return {IndicatorMode::kLeft, "turn:left"};
      }
      return {IndicatorMode::kRight, "turn:right"};
    }
  }

  return {IndicatorMode::kOff, "off"};
}

}  // namespace camrod_platform

#endif  // CAMROD_PLATFORM__LIGHT_DECISION_HPP_
