#include <gtest/gtest.h>

#include "camrod_control/safety_hold_paused_timer.hpp"

using camrod_control::gateStatusIndicatesObstacleHold;
using camrod_control::SafetyHoldPausedTimer;

// HJ_261105 - Drop-zone phase timeouts exclude safety-gate obstacle holds.
TEST(SafetyHoldPausedTimer, CountsOnlyUnheldTime) {
  SafetyHoldPausedTimer timer;
  timer.restart(100.0);
  for (double t = 100.1; t <= 110.0 + 1e-9; t += 0.1) {
    timer.observe(t, false);
  }
  EXPECT_NEAR(timer.activeElapsed(110.0), 10.0, 1e-6);
  // A 120 s obstacle hold adds nothing.
  for (double t = 110.1; t <= 230.0 + 1e-9; t += 0.1) {
    timer.observe(t, true);
  }
  EXPECT_NEAR(timer.activeElapsed(230.0), 10.0, 1e-6);
  EXPECT_NEAR(timer.pausedSeconds(), 120.0, 1e-6);
  for (double t = 230.1; t <= 240.0 + 1e-9; t += 0.1) {
    timer.observe(t, false);
  }
  EXPECT_NEAR(timer.activeElapsed(240.0), 20.0, 1e-6);
}

TEST(SafetyHoldPausedTimer, RestartClearsPausedTime) {
  SafetyHoldPausedTimer timer;
  timer.restart(0.0);
  timer.observe(50.0, true);
  timer.restart(50.0);
  EXPECT_DOUBLE_EQ(timer.pausedSeconds(), 0.0);
  timer.observe(55.0, false);
  EXPECT_DOUBLE_EQ(timer.activeElapsed(55.0), 5.0);
}

TEST(SafetyHoldPausedTimer, OnlyObstacleAndRouteHoldsPause) {
  EXPECT_TRUE(gateStatusIndicatesObstacleHold(
      "SAFETY_HOLD", "state=SAFETY_HOLD reasons=cost_stop_latched charging=false"));
  EXPECT_TRUE(gateStatusIndicatesObstacleHold(
      "SAFETY_HOLD", "state=SAFETY_HOLD reasons=cost_hold=0.7s charging=false"));
  EXPECT_TRUE(gateStatusIndicatesObstacleHold("ROUTE_SAFETY_HOLD", "state=ROUTE_SAFETY_HOLD"));
  EXPECT_FALSE(gateStatusIndicatesObstacleHold(
      "SAFETY_HOLD", "state=SAFETY_HOLD reasons=estop charging=false"));
  EXPECT_FALSE(gateStatusIndicatesObstacleHold(
      "STANDBY", "state=STANDBY reasons=engage=false(manual=false,mission=false)"));
  EXPECT_FALSE(gateStatusIndicatesObstacleHold("ENABLED", "state=ENABLED reasons=none"));
}
