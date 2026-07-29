// HH_260729 / TODOLIST 13 - Lock the steering-lag velocity envelope
// independently from CAN hardware and Ranger SDK behavior.

#include "gtest/gtest.h"
#include "ranger_base/steering_transition_policy.hpp"

namespace westonrobot
{

TEST(SteeringTransitionPolicy, StopsForLargeLagAndRestoresNearTarget)
{
  EXPECT_DOUBLE_EQ(
    SteeringTransitionVelocityScale(1.0, 0.0, true, 0.05, 0.35, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(
    SteeringTransitionVelocityScale(0.10, 0.05, true, 0.05, 0.35, 0.0), 1.0);
}

TEST(SteeringTransitionPolicy, InterpolatesAndHonorsMinimumScale)
{
  const double scale =
    SteeringTransitionVelocityScale(0.20, 0.0, true, 0.05, 0.35, 0.2);
  EXPECT_NEAR(scale, 0.6, 1.0e-9);
}

TEST(SteeringTransitionPolicy, CanBeDisabled)
{
  EXPECT_DOUBLE_EQ(
    SteeringTransitionVelocityScale(1.0, 0.0, false, 0.05, 0.35, 0.0), 1.0);
}

}  // namespace westonrobot
