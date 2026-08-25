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

TEST(RadarStopRangeFilter, IncludesFrontThirtyAndRearTenCentimeterCutoffs)
{
  // HH_260825 - FRONT1/FONT2 stop exactly 0.30 m after their measured body
  // envelopes; the rear and scalar fallback retain the 0.10 m contract.
  EXPECT_TRUE(filter::isStopCandidate(0.520, 0.020, 0.550, 0.520));
  EXPECT_FALSE(filter::isStopCandidate(0.521, 0.020, 0.550, 0.520));
  EXPECT_TRUE(filter::isStopCandidate(0.417, 0.020, 0.550, 0.417));
  EXPECT_FALSE(filter::isStopCandidate(0.418, 0.020, 0.550, 0.417));
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
