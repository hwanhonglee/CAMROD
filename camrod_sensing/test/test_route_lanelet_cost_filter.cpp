#include <cstdint>

#include <gtest/gtest.h>

#include "camrod_sensing/route_lanelet_cost_filter.hpp"

namespace
{

avg_msgs::msg::AvgOccupancyGrid makeGrid(
  const std::uint32_t width, const std::uint32_t height, const float resolution,
  const int fill_value)
{
  avg_msgs::msg::AvgOccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
    static_cast<int8_t>(fill_value));
  return grid;
}

void setCell(
  avg_msgs::msg::AvgOccupancyGrid & grid, const std::uint32_t x,
  const std::uint32_t y, const int value)
{
  grid.data.at(static_cast<std::size_t>(y) * grid.info.width + x) =
    static_cast<int8_t>(value);
}

// HH_260720 - Cells inside the selected lanelet remain obstacle costs while an
// adjacent-lane cell is removed from the sensor grid.
TEST(RouteLaneletCostFilter, KeepsInsideAndRemovesOutsideCosts)
{
  auto mask = makeGrid(6U, 4U, 1.0F, 100);
  setCell(mask, 1U, 1U, 0);
  setCell(mask, 2U, 1U, 0);

  auto obstacles = makeGrid(6U, 4U, 1.0F, 0);
  setCell(obstacles, 1U, 1U, 90);
  setCell(obstacles, 4U, 1U, 90);

  const auto removed =
    camrod::sensing::route_lanelet_cost_filter::removeCostsOutsideRouteLanelets(
      obstacles, mask, 0.0, 50, 0);

  EXPECT_EQ(removed, 1U);
  EXPECT_EQ(obstacles.data.at(7U), 90);
  EXPECT_EQ(obstacles.data.at(10U), 0);
}

// HH_260720 - The configured margin is measured from the lanelet cell edge,
// giving predictable metric behavior independent of cell-center quantization.
TEST(RouteLaneletCostFilter, AppliesMetricMarginFromLaneletBoundary)
{
  const auto mask = []() {
      auto grid = makeGrid(5U, 3U, 1.0F, 100);
      setCell(grid, 1U, 1U, 0);
      return grid;
    }();

  EXPECT_TRUE(camrod::sensing::route_lanelet_cost_filter::isWorldPointAllowed(
      mask, 2.30, 1.50, 0.30, 50));
  EXPECT_FALSE(camrod::sensing::route_lanelet_cost_filter::isWorldPointAllowed(
      mask, 2.31, 1.50, 0.30, 50));
}

// HH_260720 - Unknown startup placeholders contain no allowed route and are
// distinguishable from a valid active-route mask for fail-open node behavior.
TEST(RouteLaneletCostFilter, RejectsUnknownOrMalformedMasks)
{
  const auto unknown_mask = makeGrid(4U, 4U, 0.5F, -1);
  EXPECT_FALSE(camrod::sensing::route_lanelet_cost_filter::hasAllowedCell(
      unknown_mask, 50));

  auto malformed_mask = unknown_mask;
  malformed_mask.data.pop_back();
  EXPECT_FALSE(camrod::sensing::route_lanelet_cost_filter::hasValidGeometry(
      malformed_mask));
}

}  // namespace
