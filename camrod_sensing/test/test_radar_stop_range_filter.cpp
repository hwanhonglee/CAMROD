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

TEST(RadarStopRangeFilter, IncludesAbsoluteFrontThirtyAndSideRearTenCentimeterCutoffs)
{
  // HH_260904 - Both front sensors stop at an absolute 0.30 m range from
  // their faces. Side/rear retain the original absolute 0.10 m cutoff; the
  // rear channel is disabled separately because its chassis echo fills it.
  EXPECT_TRUE(filter::isStopCandidate(0.300, 0.020, 0.500, 0.300));
  EXPECT_FALSE(filter::isStopCandidate(0.301, 0.020, 0.500, 0.300));
  EXPECT_TRUE(filter::isStopCandidate(0.300, 0.020, 0.500, 0.300));
  EXPECT_FALSE(filter::isStopCandidate(0.301, 0.020, 0.500, 0.300));
  EXPECT_TRUE(filter::isStopCandidate(0.100, 0.020, 0.500, 0.100));
  EXPECT_FALSE(filter::isStopCandidate(0.101, 0.020, 0.500, 0.100));
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

// HH_261102 - Exercise the driver's float32 millimetre conversion rather than only double literals.
TEST(RadarStopRangeFilter, IncludesWireFloatCutoffsWithoutAdmittingTheNextMillimetre)
{
  for (const int cutoff_mm : {100, 300}) {
    const double cutoff_m = static_cast<double>(cutoff_mm) / 1000.0;
    const float wire_range_m = static_cast<float>(cutoff_mm) / 1000.0f;
    EXPECT_TRUE(filter::isStopCandidate(wire_range_m, 0.020f, 0.500f, cutoff_m));
    EXPECT_TRUE(filter::isStopCandidate(0.020f, 0.020, 0.500, cutoff_m));
    EXPECT_FALSE(filter::isStopCandidate(
      static_cast<float>(cutoff_mm + 1) / 1000.0f, 0.020f, 0.500f, cutoff_m));
  }
  EXPECT_FALSE(filter::isStopCandidate(0.019f, 0.020, 0.500, 0.300));
  EXPECT_FALSE(filter::isStopCandidate(0.501f, 0.020, 0.500, 0.500));
}
