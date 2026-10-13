// HH_261007 - Pure authored-area validation; no runtime or motion authority.

#include <algorithm>
#include <cmath>
#include <limits>

#include "camrod_control/manual_service_area.hpp"
#include "gtest/gtest.h"

namespace camrod_control {
namespace {

ManualServiceArea square() {
  return {"test_site", {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}};
}

ManualServiceArea b1() {
  // Exact corners from the authored camping_site_1 map export.
  return {"camping_site_1", {{27.3525, -5.96306}, {24.0359, -7.39992},
                             {22.3207, -3.8}, {25.6217, -2.33562}}};
}

TEST(ManualServiceArea, RealB1RejectsOutsideCenterButContainsFrontContact) {
  const auto area = b1();
  ASSERT_TRUE(validManualServiceArea(area));
  EXPECT_FALSE(manualServiceAreaContains(area, 23.026, -5.710));
  EXPECT_TRUE(manualServiceAreaContains(area, 23.792, -5.439));
  EXPECT_TRUE(manualServiceAreaContains(area, 24.835, -4.87389));
}

TEST(ManualServiceArea, ClockwiseAndCounterclockwiseAreEquivalent) {
  auto area = b1();
  auto reversed = area;
  std::reverse(reversed.polygon.begin(), reversed.polygon.end());
  ASSERT_TRUE(validManualServiceArea(reversed));
  for (const auto &point : area.polygon) {
    EXPECT_TRUE(manualServiceAreaContains(area, point.first, point.second));
    EXPECT_TRUE(manualServiceAreaContains(reversed, point.first, point.second));
  }
  for (const auto &point : {std::make_pair(23.026, -5.710),
                           std::make_pair(23.792, -5.439)}) {
    EXPECT_EQ(manualServiceAreaContains(area, point.first, point.second),
              manualServiceAreaContains(reversed, point.first, point.second));
  }
}

TEST(ManualServiceArea, IncludesEdgesAndVerticesWithOnlyNumericalTolerance) {
  const auto area = square();
  EXPECT_TRUE(manualServiceAreaContains(area, 0.0, 0.0));
  EXPECT_TRUE(manualServiceAreaContains(area, 2.0, 2.0));
  EXPECT_TRUE(manualServiceAreaContains(area, 1.0, 0.0));
  EXPECT_TRUE(manualServiceAreaContains(area, 1.0, -0.5e-8));
  EXPECT_FALSE(manualServiceAreaContains(area, 1.0, -2.0e-8));
  EXPECT_FALSE(manualServiceAreaContains(area, 2.01, 1.0));
}

TEST(ManualServiceArea, RejectsTooFewOrTooManyVertices) {
  for (std::size_t count = 0; count < 3; ++count) {
    auto area = square();
    area.polygon.resize(count);
    EXPECT_FALSE(validManualServiceArea(area));
    EXPECT_FALSE(manualServiceAreaContains(area, 0.0, 0.0));
  }
  for (const int count : {32, 33}) {
    ManualServiceArea area{"regular", {}};
    for (int i = 0; i < count; ++i) {
      const double angle = 2.0 * 3.14159265358979323846 * i / count;
      area.polygon.emplace_back(std::cos(angle), std::sin(angle));
    }
    EXPECT_EQ(validManualServiceArea(area), count == 32);
  }
}

TEST(ManualServiceArea, AcceptsTriangleAndAreaLimitButRejectsLargerArea) {
  EXPECT_TRUE(validManualServiceArea({"triangle", {{0, 0}, {2, 0}, {0, 2}}}));
  EXPECT_TRUE(validManualServiceArea(
      {"maximum", {{0, 0}, {20, 0}, {20, 20}, {0, 20}}}));
  EXPECT_FALSE(validManualServiceArea(
      {"too_large", {{0, 0}, {20.001, 0}, {20.001, 20}, {0, 20}}}));
}

TEST(ManualServiceArea, RejectsNonfiniteVerticesAndQueries) {
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
    for (const bool x_axis : {false, true}) {
      auto area = square();
      (x_axis ? area.polygon[1].first : area.polygon[1].second) = invalid;
      EXPECT_FALSE(validManualServiceArea(area));
      EXPECT_FALSE(manualServiceAreaContains(area, 1.0, 1.0));
    }
    EXPECT_FALSE(manualServiceAreaContains(square(), invalid, 1.0));
    EXPECT_FALSE(manualServiceAreaContains(square(), 1.0, invalid));
  }
}

TEST(ManualServiceArea, RejectsZeroEdgesAndDuplicateClosingVertex) {
  auto area = square();
  area.polygon[1] = area.polygon[0];
  EXPECT_FALSE(validManualServiceArea(area));
  area = square();
  area.polygon.push_back(area.polygon.front());
  EXPECT_FALSE(validManualServiceArea(area));
}

TEST(ManualServiceArea, RejectsCollinearAndDegenerateGeometry) {
  EXPECT_FALSE(validManualServiceArea({"line", {{0, 0}, {1, 0}, {2, 0}}}));
  EXPECT_FALSE(validManualServiceArea(
      {"flat_corner", {{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}}}));
  EXPECT_FALSE(validManualServiceArea(
      {"tiny", {{0, 0}, {1e-10, 0}, {1e-10, 1e-10}, {0, 1e-10}}}));
}

TEST(ManualServiceArea, RejectsConcavePolygonEvenForInteriorQuery) {
  const ManualServiceArea area{
      "concave", {{0, 0}, {2, 0}, {1, 1}, {2, 2}, {0, 2}}};
  EXPECT_FALSE(validManualServiceArea(area));
  EXPECT_FALSE(manualServiceAreaContains(area, 0.5, 1.0));
}

TEST(ManualServiceArea, RejectsSelfCrossingBowtieAndConsistentTurnStar) {
  EXPECT_FALSE(validManualServiceArea(
      {"bowtie", {{0, 0}, {2, 2}, {0, 2}, {2, 0}}}));
  ManualServiceArea star{"star", {}};
  for (const int vertex : {0, 2, 4, 1, 3}) {
    const double angle = 2.0 * 3.14159265358979323846 * vertex / 5.0;
    star.polygon.emplace_back(std::cos(angle), std::sin(angle));
  }
  EXPECT_FALSE(validManualServiceArea(star));
  EXPECT_FALSE(manualServiceAreaContains(star, 0.0, 0.0));
}

TEST(ManualServiceArea, LargeFiniteOriginDoesNotCancelSmallArea) {
  auto area = square();
  for (auto &point : area.polygon) {
    point.first += 1e8;
    point.second -= 1e8;
  }
  ASSERT_TRUE(validManualServiceArea(area));
  EXPECT_TRUE(manualServiceAreaContains(area, 1e8 + 1.0, -1e8 + 1.0));
  EXPECT_FALSE(manualServiceAreaContains(area, 1e8 + 3.0, -1e8 + 1.0));
}

TEST(ManualServiceArea, AccessIsExactUnionNotConvexHull) {
  auto area = square();
  area.access_polygons = {{{-2, 0}, {0, 0}, {0, 0.5}, {-2, 0.5}}};
  ASSERT_TRUE(validManualServiceArea(area));
  EXPECT_TRUE(manualServiceAreaContains(area, -1, 0.25));
  EXPECT_TRUE(manualServiceAreaContains(area, 1, 1));
  // Inside the hull of the two polygons, outside BOTH actual polygons.
  EXPECT_FALSE(manualServiceAreaContains(area, -0.5, 1));
  EXPECT_FALSE(manualServiceAreaContains(area, -1, -2e-8));
}

TEST(ManualServiceArea, AccessMustBeFiniteConvexAndTouchAuthoredSite) {
  auto area = square();
  for (const ManualServicePolygon &access : {
      ManualServicePolygon{{-2, 0}, {-0.1, 0}, {-0.1, 0.5}, {-2, 0.5}},
      ManualServicePolygon{{-2, 0}, {0, 0}, {-1, 0.25}, {0, 0.5}, {-2, 0.5}},
      ManualServicePolygon{{-2, 0}, {0, 0}, {0, std::numeric_limits<double>::infinity()}}}) {
    area.access_polygons = {access};
    EXPECT_FALSE(validManualServiceArea(area));
    EXPECT_FALSE(manualServiceAreaContains(area, 1, 1));
  }
}

TEST(ManualServiceArea, DetectsAccessOverlapWithAnotherSiteOrAccess) {
  auto first = square();
  first.access_polygons = {{{-2, 0}, {0, 0}, {0, 0.5}, {-2, 0.5}}};
  ManualServiceArea second{"second", {{-3,0},{-2,0},{-2,1},{-3,1}}};
  EXPECT_TRUE(manualServiceAreasOverlap(first, second));
  second.polygon = {{-4,0},{-3,0},{-3,1},{-4,1}};
  EXPECT_FALSE(manualServiceAreasOverlap(first, second));
  second.access_polygons = {{{-3,0},{-1,0},{-1,0.5},{-3,0.5}}};
  EXPECT_TRUE(manualServiceAreasOverlap(first, second));
}

}  // namespace
}  // namespace camrod_control
