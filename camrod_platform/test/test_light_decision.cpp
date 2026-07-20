// 260708 - Deterministic coverage for the exterior light decision logic.
// Mirrors the planning cmd_vel gate approach: every priority branch and the
// route sync guard are exercised without a ROS runtime.
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "camrod_platform/light_decision.hpp"

namespace camrod_platform
{
namespace
{

LightDecisionParams defaultParams()
{
  LightDecisionParams params;
  params.turn_pre_distance_m = 12.0;
  params.crab_lateral_threshold_mps = 0.05;
  return params;
}

LightDecisionInput routeInput(double route_s, std::vector<TurnSegment> segments)
{
  LightDecisionInput input;
  input.route_context_valid = true;
  input.route_s = route_s;
  input.turn_segments = std::move(segments);
  return input;
}

TEST(LightDecision, OffByDefault)
{
  const LightDecisionInput input;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
  EXPECT_EQ(result.source, "off");
}

TEST(LightDecision, PlatformEstopForcesHazard)
{
  LightDecisionInput input = routeInput(10.0, {{1.0F, 15.0F, 25.0F}});
  input.estop_platform = true;
  // HH_260721 - Exercise the explicit campsite maneuver activity contract.
  input.camping_site_maneuver_controller_active = true;
  input.cmd_vel_lateral_mps = 0.2;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kHazard);
  EXPECT_EQ(result.source, "hazard:estop_platform");
}

TEST(LightDecision, StateMachineEstopForcesHazard)
{
  LightDecisionInput input;
  input.estop_state_machine = true;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kHazard);
  EXPECT_EQ(result.source, "hazard:estop_state_machine");
}

TEST(LightDecision, CrabLeftWhenManeuverActiveAndLateralPositive)
{
  LightDecisionInput input;
  input.camping_site_maneuver_controller_active = true;
  input.cmd_vel_lateral_mps = 0.24;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kLeft);
  EXPECT_EQ(result.source, "crab:left");
}

TEST(LightDecision, CrabRightWhenLateralNegative)
{
  LightDecisionInput input;
  input.camping_site_maneuver_controller_active = true;
  input.cmd_vel_lateral_mps = -0.24;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kRight);
  EXPECT_EQ(result.source, "crab:right");
}

TEST(LightDecision, CrabIgnoredBelowLateralThreshold)
{
  LightDecisionInput input;
  input.camping_site_maneuver_controller_active = true;
  input.cmd_vel_lateral_mps = 0.03;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
}

TEST(LightDecision, LateralWithoutManeuverDoesNotSignal)
{
  LightDecisionInput input;
  input.camping_site_maneuver_controller_active = false;
  input.cmd_vel_lateral_mps = 0.24;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
}

TEST(LightDecision, CrabBeatsTurnWindow)
{
  LightDecisionInput input = routeInput(20.0, {{1.0F, 15.0F, 25.0F}});
  input.camping_site_maneuver_controller_active = true;
  input.cmd_vel_lateral_mps = -0.24;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kRight);
  EXPECT_EQ(result.source, "crab:right");
}

TEST(LightDecision, TurnLeftInsidePreDistanceWindow)
{
  // Segment starts at 50 m; robot at 40 m is inside the 12 m pre-window.
  const auto input = routeInput(40.0, {{1.0F, 50.0F, 60.0F}});
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kLeft);
  EXPECT_EQ(result.source, "turn:left");
}

TEST(LightDecision, NoTurnBeforePreDistanceWindow)
{
  const auto input = routeInput(37.9, {{1.0F, 50.0F, 60.0F}});
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
}

TEST(LightDecision, TurnRightInsideSegment)
{
  const auto input = routeInput(55.0, {{-1.0F, 50.0F, 60.0F}});
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kRight);
  EXPECT_EQ(result.source, "turn:right");
}

TEST(LightDecision, OffAfterSegmentEnd)
{
  const auto input = routeInput(60.1, {{-1.0F, 50.0F, 60.0F}});
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
}

TEST(LightDecision, EarliestSegmentWinsWhenWindowsOverlap)
{
  // Robot sits in the pre-window of both segments; the nearer start wins.
  const auto input = routeInput(
    48.0, {{-1.0F, 55.0F, 65.0F}, {1.0F, 50.0F, 54.0F}});
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kLeft);
}

TEST(LightDecision, InvalidRouteContextSuppressesTurn)
{
  LightDecisionInput input = routeInput(55.0, {{1.0F, 50.0F, 60.0F}});
  input.route_context_valid = false;
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
}

TEST(LightDecision, UnknownRouteSSuppressesTurn)
{
  LightDecisionInput input = routeInput(-1.0, {{1.0F, 0.0F, 60.0F}});
  const auto result = decideIndicator(input, defaultParams());
  EXPECT_EQ(result.mode, IndicatorMode::kOff);
}

TEST(LightDecision, RouteLengthGuardAcceptsWithinTolerance)
{
  EXPECT_TRUE(routeLengthMatches(120.0, 120.9, 1.0));
  EXPECT_TRUE(routeLengthMatches(120.9, 120.0, 1.0));
}

TEST(LightDecision, RouteLengthGuardRejectsBeyondTolerance)
{
  EXPECT_FALSE(routeLengthMatches(120.0, 121.1, 1.0));
  EXPECT_FALSE(routeLengthMatches(80.0, 120.0, 1.0));
}

}  // namespace
}  // namespace camrod_platform

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
