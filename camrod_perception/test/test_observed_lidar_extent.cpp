#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "camrod_perception/observed_lidar_extent.hpp"

namespace
{
struct Point
{
  double z, lx, ly, lz;
};

// HH_261002 - Display extents must reflect only observed finite foreground
// returns, preserve association buffers and omit unsupported physical sizes.
TEST(ObservedLidarExtent, ReportsUnpaddedMinMaxWithoutChangingInput)
{
  const std::vector<Point> points{
    {4.1, 4.1, -0.4, -0.6}, {4.3, 4.3, 0.2, 0.5}, {4.2, 4.2, 0.0, 0.8}};
  const auto extent = camrod_perception::EstimateObservedLidarExtent(points, 3, 0.6);
  ASSERT_TRUE(extent);
  EXPECT_EQ(extent->point_count, 3u);
  EXPECT_NEAR(extent->center[0], 4.2, 1e-9);
  EXPECT_NEAR(extent->center[1], -0.1, 1e-9);
  EXPECT_NEAR(extent->center[2], 0.1, 1e-9);
  EXPECT_NEAR(extent->size[0], 0.2, 1e-9);
  EXPECT_NEAR(extent->size[1], 0.6, 1e-9);
  EXPECT_NEAR(extent->size[2], 1.4, 1e-9);
  EXPECT_EQ(points[0].z, 4.1);
  EXPECT_EQ(points[1].z, 4.3);
}

TEST(ObservedLidarExtent, RejectsBackgroundInsideTheSameImageBox)
{
  const std::vector<Point> points{
    {4.1, 4.1, -0.4, -0.6}, {4.3, 4.3, 0.2, 0.5}, {4.2, 4.2, 0.0, 0.8},
    {6.0, 6.0, -4.0, -3.0}, {8.0, 8.0, 7.0, 6.0}};
  const auto extent = camrod_perception::EstimateObservedLidarExtent(points, 3, 0.6);
  ASSERT_TRUE(extent);
  EXPECT_EQ(extent->point_count, 3u);
  EXPECT_NEAR(extent->size[0], 0.2, 1e-9);
  EXPECT_NEAR(extent->size[1], 0.6, 1e-9);
  EXPECT_NEAR(extent->size[2], 1.4, 1e-9);
}

TEST(ObservedLidarExtent, DoesNotJumpFromSparseForegroundToBackground)
{
  const std::vector<Point> points{
    {2.0, 2.0, -0.1, 0.3}, {2.1, 2.1, 0.1, 0.6},
    {4.1, 4.1, -0.4, -0.6}, {4.3, 4.3, 0.2, 0.5}, {4.2, 4.2, 0.0, 0.8}};
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(points, 3, 0.6));
}

TEST(ObservedLidarExtent, RequiresFinitePositiveExtentsAndMinimumSupport)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<Point> invalid{
    {4.0, 4.0, 0.1, 0.0}, {4.1, 4.1, -0.1, 0.3},
    {nan, 4.2, 0.2, 0.6}, {4.3, inf, 0.3, 0.7}, {4.4, 4.4, nan, 0.8},
    {4.5, 4.5, 0.5, inf}, {-1.0, -1.0, 0.2, 0.7}};
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(invalid, 1, 0.6));
  const std::vector<Point> planar{
    {4.0, 4.0, 0.0, 0.0}, {4.1, 4.1, 0.0, 0.3}, {4.2, 4.2, 0.0, 0.6}};
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(planar, 3, 0.6));
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(planar, 3, nan));
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(planar, 3, -1.0));
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(planar, 3, inf));
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(std::vector<Point>{}, 3, 0.6));
}

TEST(ObservedLidarExtent, PreservesSmallMeasuredSizesInsteadOfPadding)
{
  const std::vector<Point> points{
    {4.000, 4.000, 0.001, 0.002}, {4.005, 4.005, 0.003, 0.004},
    {4.010, 4.010, 0.005, 0.006}};
  const auto extent = camrod_perception::EstimateObservedLidarExtent(points, 3, 0.6);
  ASSERT_TRUE(extent);
  EXPECT_NEAR(extent->size[0], 0.010, 1e-9);
  EXPECT_NEAR(extent->size[1], 0.004, 1e-9);
  EXPECT_NEAR(extent->size[2], 0.004, 1e-9);
  EXPECT_FALSE(camrod_perception::EstimateObservedLidarExtent(points, 4, 0.6));
}
}  // namespace
