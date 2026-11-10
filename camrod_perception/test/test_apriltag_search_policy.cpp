#include <gtest/gtest.h>

#include "camrod_perception/apriltag_search_policy.hpp"

using camrod_perception::AprilTagSearchPolicy;
using camrod_perception::quadDecimateFor;

// HJ_261110 - Idle frames are strided; tracked frames are never skipped.
TEST(AprilTagSearchPolicy, IdleSearchesEveryOtherFrame)
{
  AprilTagSearchPolicy policy(2);
  int processed = 0;
  for (int frame = 0; frame < 10; ++frame) {
    processed += policy.processFrame(false) ? 1 : 0;
  }
  EXPECT_EQ(processed, 5);
}

TEST(AprilTagSearchPolicy, TrackingProcessesEveryFrameAndResetsStride)
{
  AprilTagSearchPolicy policy(2);
  EXPECT_TRUE(policy.processFrame(false));
  EXPECT_FALSE(policy.processFrame(false));
  for (int frame = 0; frame < 5; ++frame) {
    EXPECT_TRUE(policy.processFrame(true));
  }
  // The first frame after losing the tag is searched immediately.
  EXPECT_TRUE(policy.processFrame(false));
  EXPECT_FALSE(policy.processFrame(false));
}

TEST(AprilTagSearchPolicy, StrideBelowOneProcessesEveryFrame)
{
  AprilTagSearchPolicy policy(0);
  EXPECT_EQ(policy.idleFrameStride(), 1);
  EXPECT_TRUE(policy.processFrame(false));
  EXPECT_TRUE(policy.processFrame(false));
}

TEST(AprilTagSearchPolicy, FullSearchIsDecimatedRoiIsPrecise)
{
  EXPECT_FLOAT_EQ(quadDecimateFor(true, 1.0, 2.0), 1.0F);
  EXPECT_FLOAT_EQ(quadDecimateFor(false, 1.0, 2.0), 2.0F);
  EXPECT_FLOAT_EQ(quadDecimateFor(false, 1.0, 0.5), 1.0F);
}
