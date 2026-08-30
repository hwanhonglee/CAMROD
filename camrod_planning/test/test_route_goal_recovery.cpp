// HH_260729 / TODOLIST 11-12 - Verify that route recovery refreshes a retained
// active or aborted Nav2 goal, cannot restart a terminal goal, and remains
// delayed/bounded per goal.

#include "camrod_planning/route_goal_recovery.hpp"
#include "gtest/gtest.h"

namespace camrod_planning
{

TEST(RouteGoalRecovery, AbortedGoalRequiresHoldAndEnabledClearDelay)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.5;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();

  recovery.observeNavAborted();
  recovery.observeRouteHold(false, true, 1.0);
  EXPECT_FALSE(recovery.ready(2.0));

  recovery.observeRouteHold(true, false, 2.0);
  recovery.observeRouteHold(false, false, 2.5);
  EXPECT_FALSE(recovery.ready(3.0));

  recovery.observeRouteHold(false, true, 3.0);
  EXPECT_FALSE(recovery.ready(3.49));
  EXPECT_TRUE(recovery.ready(3.5));
}

TEST(RouteGoalRecovery, ActiveGoalReissuesAfterHoldAndEnabledClearDelay)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.5;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();

  recovery.observeNavActive();
  recovery.observeRouteHold(true, false, 1.0);
  EXPECT_FALSE(recovery.ready(1.5));

  recovery.observeRouteHold(false, true, 2.0);
  EXPECT_FALSE(recovery.ready(2.49));
  EXPECT_TRUE(recovery.ready(2.5));
  EXPECT_TRUE(recovery.navActive());
  EXPECT_FALSE(recovery.navAborted());
}

TEST(RouteGoalRecovery, ActiveGoalWithoutHoldNeverReissues)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.0;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();
  recovery.observeNavActive();

  recovery.observeRouteHold(false, true, 1.0);

  EXPECT_FALSE(recovery.routeHoldSeen());
  EXPECT_FALSE(recovery.ready(10.0));
}

TEST(RouteGoalRecovery, ReissueIsIntervalBoundedAndNeedsANewHold)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.0;
  config.minimum_reissue_interval_s = 2.0;
  config.maximum_reissues_per_goal = 2;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();
  recovery.observeNavActive();
  recovery.observeRouteHold(true, false, 1.0);
  recovery.observeRouteHold(false, true, 1.1);
  ASSERT_TRUE(recovery.ready(1.1));
  recovery.markReissued(1.1);

  EXPECT_EQ(recovery.reissueCount(), 1);
  EXPECT_FALSE(recovery.routeHoldSeen());
  EXPECT_FALSE(recovery.navActive());
  EXPECT_FALSE(recovery.ready(2.0));

  recovery.observeNavActive();
  EXPECT_FALSE(recovery.ready(1.2));
  recovery.observeRouteHold(true, false, 2.0);
  recovery.observeRouteHold(false, true, 2.1);
  EXPECT_FALSE(recovery.ready(3.09));
  ASSERT_TRUE(recovery.ready(3.11));
  recovery.markReissued(3.11);

  recovery.observeNavActive();
  recovery.observeRouteHold(true, false, 4.0);
  recovery.observeRouteHold(false, true, 4.1);
  EXPECT_FALSE(recovery.ready(6.0));
}

TEST(RouteGoalRecovery, SucceededStatusSuppressesLaterReengage)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.0;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();
  recovery.observeRouteHold(true, false, 1.0);
  recovery.observeNavActive();
  recovery.observeNavSucceeded();
  recovery.observeRouteHold(false, true, 2.0);

  // A late/stale active status cannot revive the terminal retained goal.
  recovery.observeNavActive();

  EXPECT_FALSE(recovery.routeHoldSeen());
  EXPECT_FALSE(recovery.navActive());
  EXPECT_FALSE(recovery.navAborted());
  EXPECT_FALSE(recovery.ready(3.0));
}

TEST(RouteGoalRecovery, CanceledStatusSuppressesLaterReengage)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.0;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();
  recovery.observeRouteHold(true, false, 1.0);
  recovery.observeNavActive();

  recovery.observeNavCanceled();
  recovery.observeRouteHold(false, true, 2.0);
  recovery.observeNavActive();

  EXPECT_FALSE(recovery.routeHoldSeen());
  EXPECT_FALSE(recovery.navActive());
  EXPECT_FALSE(recovery.navAborted());
  EXPECT_FALSE(recovery.ready(3.0));
}

}  // namespace camrod_planning
