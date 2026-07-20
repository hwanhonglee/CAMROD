#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <avg_msgs/msg/avg_occupancy_grid.hpp>

namespace camrod::sensing::route_lanelet_cost_filter
{

// HH_260720 - Keep route-lanelet mask geometry in one testable implementation
// shared by LiDAR and radar cost-grid publishers.
inline bool hasValidGeometry(const avg_msgs::msg::AvgOccupancyGrid & grid)
{
  const auto expected_size =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  return grid.info.resolution > 0.0F && grid.info.width > 0U && grid.info.height > 0U &&
         grid.data.size() == expected_size;
}

inline double originYaw(const avg_msgs::msg::AvgOccupancyGrid & grid)
{
  const auto & q = grid.info.origin.orientation;
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}

inline bool worldToGridLocal(
  const avg_msgs::msg::AvgOccupancyGrid & grid, const double world_x, const double world_y,
  double & local_x, double & local_y)
{
  if (!hasValidGeometry(grid) || !std::isfinite(world_x) || !std::isfinite(world_y)) {
    return false;
  }
  const double yaw = originYaw(grid);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double dx = world_x - grid.info.origin.position.x;
  const double dy = world_y - grid.info.origin.position.y;
  local_x = cos_yaw * dx + sin_yaw * dy;
  local_y = -sin_yaw * dx + cos_yaw * dy;
  return true;
}

inline bool hasAllowedCell(
  const avg_msgs::msg::AvgOccupancyGrid & mask, const int allowed_max_cost)
{
  if (!hasValidGeometry(mask)) {
    return false;
  }
  return std::any_of(mask.data.begin(), mask.data.end(), [allowed_max_cost](const int8_t value) {
      return value >= 0 && static_cast<int>(value) <= allowed_max_cost;
    });
}

inline bool isWorldPointAllowed(
  const avg_msgs::msg::AvgOccupancyGrid & mask, const double world_x, const double world_y,
  const double margin_m, const int allowed_max_cost)
{
  double local_x = 0.0;
  double local_y = 0.0;
  if (!worldToGridLocal(mask, world_x, world_y, local_x, local_y)) {
    return false;
  }

  const double resolution = static_cast<double>(mask.info.resolution);
  const double margin = std::max(0.0, margin_m);
  const int center_x = static_cast<int>(std::floor(local_x / resolution));
  const int center_y = static_cast<int>(std::floor(local_y / resolution));
  const int search_radius = static_cast<int>(std::ceil(margin / resolution)) + 1;

  for (int cell_y = center_y - search_radius; cell_y <= center_y + search_radius; ++cell_y) {
    if (cell_y < 0 || cell_y >= static_cast<int>(mask.info.height)) {
      continue;
    }
    for (int cell_x = center_x - search_radius; cell_x <= center_x + search_radius; ++cell_x) {
      if (cell_x < 0 || cell_x >= static_cast<int>(mask.info.width)) {
        continue;
      }
      const auto index = static_cast<std::size_t>(cell_y) * mask.info.width +
        static_cast<std::size_t>(cell_x);
      const int value = static_cast<int>(mask.data[index]);
      if (value < 0 || value > allowed_max_cost) {
        continue;
      }

      // HH_260720 - Measure to the allowed cell rectangle, rather than its
      // center, so margin=0 means exactly the rasterized lanelet interior.
      const double cell_min_x = static_cast<double>(cell_x) * resolution;
      const double cell_max_x = cell_min_x + resolution;
      const double cell_min_y = static_cast<double>(cell_y) * resolution;
      const double cell_max_y = cell_min_y + resolution;
      const double distance_x = std::max({cell_min_x - local_x, 0.0, local_x - cell_max_x});
      const double distance_y = std::max({cell_min_y - local_y, 0.0, local_y - cell_max_y});
      if (std::hypot(distance_x, distance_y) <= margin + 1e-9) {
        return true;
      }
    }
  }
  return false;
}

inline std::size_t removeCostsOutsideRouteLanelets(
  avg_msgs::msg::AvgOccupancyGrid & obstacle_grid,
  const avg_msgs::msg::AvgOccupancyGrid & route_lanelet_mask, const double margin_m,
  const int allowed_max_cost, const int free_value)
{
  if (!hasValidGeometry(obstacle_grid) || !hasValidGeometry(route_lanelet_mask)) {
    return 0U;
  }

  const double source_resolution = static_cast<double>(obstacle_grid.info.resolution);
  const double source_yaw = originYaw(obstacle_grid);
  const double cos_yaw = std::cos(source_yaw);
  const double sin_yaw = std::sin(source_yaw);
  std::size_t removed_count = 0U;

  for (std::size_t index = 0; index < obstacle_grid.data.size(); ++index) {
    if (static_cast<int>(obstacle_grid.data[index]) <= free_value) {
      continue;
    }
    const auto cell_x = static_cast<std::uint32_t>(index % obstacle_grid.info.width);
    const auto cell_y = static_cast<std::uint32_t>(index / obstacle_grid.info.width);
    const double local_x = (static_cast<double>(cell_x) + 0.5) * source_resolution;
    const double local_y = (static_cast<double>(cell_y) + 0.5) * source_resolution;
    const double world_x = obstacle_grid.info.origin.position.x +
      cos_yaw * local_x - sin_yaw * local_y;
    const double world_y = obstacle_grid.info.origin.position.y +
      sin_yaw * local_x + cos_yaw * local_y;
    if (!isWorldPointAllowed(
        route_lanelet_mask, world_x, world_y, margin_m, allowed_max_cost))
    {
      obstacle_grid.data[index] = static_cast<int8_t>(std::clamp(free_value, -1, 100));
      ++removed_count;
    }
  }
  return removed_count;
}

}  // namespace camrod::sensing::route_lanelet_cost_filter
