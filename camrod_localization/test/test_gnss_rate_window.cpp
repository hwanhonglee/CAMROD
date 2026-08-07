#include <gtest/gtest.h>

#include "camrod_localization/gnss_rate_window.hpp"

namespace
{

using camrod_localization::GnssRateWindow;

TEST(GnssRateWindow, DeduplicatesPoseAndCovarianceMirrors)
{
  GnssRateWindow window(2.0);

  EXPECT_TRUE(window.record(1'000'000'000));
  EXPECT_FALSE(window.record(1'000'000'000));
  EXPECT_EQ(window.sampleCount(), 1U);
  EXPECT_FALSE(window.ready());
}

TEST(GnssRateWindow, IgnoresLateEpochWithoutRewindingHistory)
{
  GnssRateWindow window(2.0);

  EXPECT_TRUE(window.record(1'000'000'000));
  EXPECT_TRUE(window.record(1'200'000'000));
  EXPECT_FALSE(window.record(1'100'000'000));
  EXPECT_EQ(window.sampleCount(), 2U);
  EXPECT_DOUBLE_EQ(window.rateHz(), 5.0);
}

TEST(GnssRateWindow, UsesFreshnessUntilEnoughUniqueEpochsArrive)
{
  GnssRateWindow window(2.0);

  EXPECT_TRUE(window.record(0));
  EXPECT_TRUE(window.record(200'000'000));
  EXPECT_FALSE(window.ready());
  EXPECT_DOUBLE_EQ(window.rateHz(), 5.0);

  EXPECT_TRUE(window.record(400'000'000));
  EXPECT_TRUE(window.ready());
  EXPECT_DOUBLE_EQ(window.rateHz(), 5.0);
}

TEST(GnssRateWindow, OneMissedFiveHertzEpochDoesNotLookLikeOneHertz)
{
  GnssRateWindow window(2.0);

  EXPECT_TRUE(window.record(0));
  EXPECT_TRUE(window.record(200'000'000));
  EXPECT_TRUE(window.record(400'000'000));
  EXPECT_TRUE(window.record(800'000'000));

  EXPECT_TRUE(window.ready());
  EXPECT_DOUBLE_EQ(window.rateHz(), 3.75);
}

TEST(GnssRateWindow, SustainedOneHertzStillFailsThreeHertzFloor)
{
  GnssRateWindow window(2.0);

  EXPECT_TRUE(window.record(0));
  EXPECT_TRUE(window.record(1'000'000'000));
  EXPECT_TRUE(window.record(2'000'000'000));

  EXPECT_TRUE(window.ready());
  EXPECT_DOUBLE_EQ(window.rateHz(), 1.0);
  EXPECT_LT(window.rateHz(), 3.0);
}

}  // namespace
