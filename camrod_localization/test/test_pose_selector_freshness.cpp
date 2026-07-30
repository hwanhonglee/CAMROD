#include <gtest/gtest.h>

#include "camrod_localization/pose_selector_freshness.hpp"

namespace
{

using camrod_localization::PosePayload;
using camrod_localization::selectFreshestPosePayload;

TEST(PoseSelectorFreshness, UsesOdometryWhenMatchingPoseHasPreviousCycleStamp)
{
  EXPECT_EQ(
    selectFreshestPosePayload(true, 1'000'000'000, true, 1'050'000'000),
    PosePayload::kOdometry);
}

TEST(PoseSelectorFreshness, UsesNewPoseCovarianceWhenItArrivesFirst)
{
  EXPECT_EQ(
    selectFreshestPosePayload(true, 1'050'000'000, true, 1'000'000'000),
    PosePayload::kPoseCovariance);
}

TEST(PoseSelectorFreshness, PrefersPoseCovarianceForEqualStamp)
{
  EXPECT_EQ(
    selectFreshestPosePayload(true, 1'050'000'000, true, 1'050'000'000),
    PosePayload::kPoseCovariance);
}

TEST(PoseSelectorFreshness, SupportsSingleInputAndNoInput)
{
  EXPECT_EQ(
    selectFreshestPosePayload(true, 1'000'000'000, false, 0),
    PosePayload::kPoseCovariance);
  EXPECT_EQ(
    selectFreshestPosePayload(false, 0, true, 1'000'000'000),
    PosePayload::kOdometry);
  EXPECT_EQ(
    selectFreshestPosePayload(false, 0, false, 0),
    PosePayload::kNone);
}

}  // namespace
