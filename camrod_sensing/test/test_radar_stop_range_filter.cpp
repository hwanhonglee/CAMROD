#include <gtest/gtest.h>

#include <limits>

#include "camrod_sensing/radar_stop_range_filter.hpp"

namespace filter = camrod::sensing::radar_stop_range_filter;

TEST(RadarStopRangeFilter, KeepsObservationAndStopRangesIndependent)
{
  EXPECT_TRUE(filter::isStopCandidate(0.080, 0.020, 0.500, 0.100));
  EXPECT_TRUE(filter::isStopCandidate(0.100, 0.020, 0.500, 0.100));
  EXPECT_FALSE(filter::isStopCandidate(0.101, 0.020, 0.500, 0.100));
  EXPECT_FALSE(filter::isStopCandidate(0.400, 0.020, 0.500, 0.100));
}

TEST(RadarStopRangeFilter, IncludesAbsoluteFrontThirtyAndRearBodyRelativeCutoffs)
{
  // HH_260904 - Both front sensors stop at an absolute 0.30 m range from
  // their faces. Rear retains its measured body edge plus 0.10 m window.
  EXPECT_TRUE(filter::isStopCandidate(0.300, 0.020, 0.500, 0.300));
  EXPECT_FALSE(filter::isStopCandidate(0.301, 0.020, 0.500, 0.300));
  EXPECT_TRUE(filter::isStopCandidate(0.300, 0.020, 0.500, 0.300));
  EXPECT_FALSE(filter::isStopCandidate(0.301, 0.020, 0.500, 0.300));
  EXPECT_TRUE(filter::isStopCandidate(0.206, 0.020, 0.500, 0.206));
  EXPECT_FALSE(filter::isStopCandidate(0.207, 0.020, 0.500, 0.206));
}

TEST(RadarStopRangeFilter, RejectsInvalidAndNoTargetSamples)
{
  EXPECT_FALSE(filter::isStopCandidate(0.019, 0.020, 0.500, 0.100));
  EXPECT_FALSE(filter::isStopCandidate(0.501, 0.020, 0.500, 0.100));
  EXPECT_FALSE(filter::isStopCandidate(
    std::numeric_limits<double>::quiet_NaN(), 0.020, 0.500, 0.100));
  EXPECT_FALSE(filter::isStopCandidate(
    std::numeric_limits<double>::infinity(), 0.020, 0.500, 0.100));
}
