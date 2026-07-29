// HH_260729 / TODOLIST 11-12 - Verify that route recovery cannot restart
// canceled or unrelated planning failures and remains bounded for a retained
// goal.

#include "camrod_planning/route_goal_recovery.hpp"
#include "gtest/gtest.h"

namespace camrod_planning
{

TEST(RouteGoalRecovery, RequiresHoldAbortAndEnabledClearDelay)
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

TEST(RouteGoalRecovery, ReissueIsBoundedAndNeedsANewHold)
{
  RouteGoalRecoveryConfig config;
  config.clear_delay_s = 0.0;
  config.minimum_reissue_interval_s = 0.0;
  config.maximum_reissues_per_goal = 1;
  RouteGoalRecovery recovery(config);
  recovery.resetForGoal();
  recovery.observeRouteHold(true, false, 1.0);
  recovery.observeNavAborted();
  recovery.observeRouteHold(false, true, 1.1);
  ASSERT_TRUE(recovery.ready(1.1));
  recovery.markReissued(1.1);

  EXPECT_EQ(recovery.reissueCount(), 1);
  EXPECT_FALSE(recovery.ready(2.0));
  recovery.observeRouteHold(true, false, 2.0);
  recovery.observeNavAborted();
  recovery.observeRouteHold(false, true, 2.1);
  EXPECT_FALSE(recovery.ready(2.1));
}

TEST(RouteGoalRecovery, ActiveOrSucceededStatusClearsAbort)
{
  RouteGoalRecovery recovery;
  recovery.resetForGoal();
  recovery.observeRouteHold(true, false, 1.0);
  recovery.observeNavAborted();
  recovery.observeNavActiveOrSucceeded();
  recovery.observeRouteHold(false, true, 2.0);
  EXPECT_FALSE(recovery.ready(3.0));
}

}  // namespace camrod_planning
