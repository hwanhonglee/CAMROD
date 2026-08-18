// HH_260729 / TODOLIST 13 - Lock the steering-lag velocity envelope
// independently from CAN hardware and Ranger SDK behavior.

#include "gtest/gtest.h"
#include "ranger_base/parallel_motion_policy.hpp"
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

TEST(SteeringTransitionPolicy, ModeChangeOverridesOrdinaryVelocityFloor)
{
  const double regular_scale =
    SteeringTransitionVelocityScale(1.57, 0.0, true, 0.05, 0.70, 0.20);
  ASSERT_DOUBLE_EQ(regular_scale, 0.20);
  EXPECT_DOUBLE_EQ(
    SteeringModeTransitionVelocityScale(
      regular_scale, true, 1.57, 0.0, 0.05),
    0.0);
}

TEST(SteeringTransitionPolicy, ModeChangeReleasesOnlyAtRequestedAngle)
{
  EXPECT_DOUBLE_EQ(
    SteeringModeTransitionVelocityScale(0.75, true, 1.57, 1.50, 0.05),
    0.0);
  EXPECT_DOUBLE_EQ(
    SteeringModeTransitionVelocityScale(1.0, true, 1.57, 1.53, 0.05),
    1.0);
  EXPECT_DOUBLE_EQ(
    SteeringModeTransitionVelocityScale(0.6, false, 0.2, 0.0, 0.05),
    0.6);
}

TEST(SteeringTransitionPolicy, IntermediateAngleCannotReleaseInitialParallelRequest)
{
  // HH_260818 - 0.775 rad can exceed the Ackermann limit after one limiter
  // step, but it is still far from the requested 90 degree crab geometry.
  EXPECT_DOUBLE_EQ(
    SteeringModeTransitionVelocityScale(0.20, true, 1.57, 0.775, 0.05),
    0.0);
}

TEST(ParallelMotionPolicy, PreservesRequestedVectorInAllQuadrants)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  const struct
  {
    double x;
    double y;
  } samples[] = {
    {1.0, 1.0},
    {-1.0, 1.0},
    {-1.0, -1.0},
    {1.0, -1.0},
    {0.0, 1.0},
    {0.0, -1.0},
  };

  for (const auto & sample : samples) {
    const auto command = ResolveParallelMotionCommand(sample.x, sample.y);
    EXPECT_GE(command.steering_angle_rad, -kHalfPi);
    EXPECT_LE(command.steering_angle_rad, kHalfPi);
    EXPECT_NEAR(
      command.signed_speed * std::cos(command.steering_angle_rad),
      sample.x, 1.0e-9);
    EXPECT_NEAR(
      command.signed_speed * std::sin(command.steering_angle_rad),
      sample.y, 1.0e-9);
  }
}

TEST(ParallelMotionPolicy, PureCrabDoesNotDependOnEarlierLongitudinalDirection)
{
  const auto left = ResolveParallelMotionCommand(0.0, 0.4);
  const auto right = ResolveParallelMotionCommand(0.0, -0.4);

  EXPECT_NEAR(left.signed_speed * std::sin(left.steering_angle_rad), 0.4, 1.0e-9);
  EXPECT_NEAR(right.signed_speed * std::sin(right.steering_angle_rad), -0.4, 1.0e-9);
}

TEST(ParallelMotionPolicy, LateralResidueDoesNotSelectCrabMode)
{
  // HH_260818 - Ordinary path tracking must not pay the stationary steering
  // transition penalty for tiny bridge/controller lateral residue.
  EXPECT_FALSE(ShouldUseParallelMotion(0.0, 0.02));
  EXPECT_FALSE(ShouldUseParallelMotion(0.019, 0.02));
  EXPECT_FALSE(ShouldUseParallelMotion(-0.02, 0.02));
  EXPECT_TRUE(ShouldUseParallelMotion(0.021, 0.02));
  EXPECT_TRUE(ShouldUseParallelMotion(-0.10, 0.02));
  EXPECT_TRUE(ShouldUseParallelMotion(0.666667, 0.02));
}

}  // namespace westonrobot
