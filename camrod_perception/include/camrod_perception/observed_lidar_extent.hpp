#ifndef CAMROD_PERCEPTION__OBSERVED_LIDAR_EXTENT_HPP_
#define CAMROD_PERCEPTION__OBSERVED_LIDAR_EXTENT_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace camrod_perception
{

// HH_261002 - This is the extent of visible foreground returns, not an inferred
// full object box. Camera pixels, class priors and display padding are never
// converted to physical metres. The axes remain those of the source LiDAR.
struct ObservedLidarExtent
{
  std::array<double, 3> center;
  std::array<double, 3> size;
  std::size_t point_count;
};

// Point supplies camera depth z and raw LiDAR coordinates lx/ly/lz, matching the
// existing fusion association buffer. Two linear scans add no point allocation
// and do not reorder or modify the buffer used by tracking and safety outputs.
template<typename Point>
std::optional<ObservedLidarExtent> EstimateObservedLidarExtent(
  const std::vector<Point> & points, std::size_t min_points, double depth_band_m)
{
  if (!std::isfinite(depth_band_m) || depth_band_m <= 0.0) {
    return std::nullopt;
  }
  min_points = std::max<std::size_t>(3, min_points);
  const auto valid = [](const Point & point) {
      return std::isfinite(point.z) && point.z > 0.0 &&
             std::isfinite(point.lx) && std::isfinite(point.ly) &&
             std::isfinite(point.lz);
    };
  double nearest_depth = std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    if (valid(point)) {
      nearest_depth = std::min(nearest_depth, static_cast<double>(point.z));
    }
  }
  if (!std::isfinite(nearest_depth)) {
    return std::nullopt;
  }

  // HH_261002 - A fixed narrow foreground slab rejects the distant background
  // inside a 2D detection. Sparse/flat support yields no box rather than a
  // fabricated body size. A near stray return can suppress an extent; it must
  // never make us jump to a larger background cluster or widen the depth gate.
  std::array<double, 3> low{
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};
  std::array<double, 3> high{-low[0], -low[1], -low[2]};
  std::size_t count = 0;
  for (const auto & point : points) {
    if (!valid(point) || static_cast<double>(point.z) - nearest_depth > depth_band_m) {
      continue;
    }
    const std::array<double, 3> position{point.lx, point.ly, point.lz};
    for (std::size_t axis = 0; axis < position.size(); ++axis) {
      low[axis] = std::min(low[axis], position[axis]);
      high[axis] = std::max(high[axis], position[axis]);
    }
    ++count;
  }
  if (count < min_points) {
    return std::nullopt;
  }
  ObservedLidarExtent extent{};
  extent.point_count = count;
  for (std::size_t axis = 0; axis < low.size(); ++axis) {
    extent.center[axis] = low[axis] + (high[axis] - low[axis]) * 0.5;
    extent.size[axis] = high[axis] - low[axis];
    if (!std::isfinite(extent.center[axis]) || !std::isfinite(extent.size[axis]) ||
      extent.size[axis] <= 0.0)
    {
      return std::nullopt;
    }
  }
  return extent;
}

}  // namespace camrod_perception

#endif  // CAMROD_PERCEPTION__OBSERVED_LIDAR_EXTENT_HPP_
