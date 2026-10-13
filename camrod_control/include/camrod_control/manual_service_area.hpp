#pragma once

// HH_261007 - Exact authored geometry only; manual ownership and fresh-pose
// authorization remain the caller's responsibility. No radius or buffer exists.

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace camrod_control {

using ManualServicePolygon = std::vector<std::pair<double, double>>;

struct ManualServiceArea {
  std::string id;
  ManualServicePolygon polygon;
  // Separately authored connectors, never a convex hull or buffered site.
  std::vector<ManualServicePolygon> access_polygons{};
};

namespace manual_service_area_detail {

constexpr long double kBoundaryEpsilonM = 1.0e-8L;

inline long double cross(const std::pair<double, double> &a,
                         const std::pair<double, double> &b,
                         const long double x, const long double y) {
  return (static_cast<long double>(b.first) - a.first) * (y - a.second) -
         (static_cast<long double>(b.second) - a.second) * (x - a.first);
}

inline long double twiceSignedArea(const ManualServicePolygon &polygon) {
  // Translate before summation to avoid cancellation from a large map origin.
  const auto &origin = polygon.front();
  long double result = 0.0L;
  for (std::size_t i = 1; i + 1 < polygon.size(); ++i) {
    result += cross(origin, polygon[i], polygon[i + 1].first,
                    polygon[i + 1].second);
  }
  return result;
}

inline long double edgeLength(const std::pair<double, double> &a,
                              const std::pair<double, double> &b) {
  return std::hypot(static_cast<long double>(b.first) - a.first,
                    static_cast<long double>(b.second) - a.second);
}

}  // namespace manual_service_area_detail

inline bool validManualServicePolygon(const ManualServicePolygon &polygon) {
  using namespace manual_service_area_detail;
  const std::size_t count = polygon.size();
  if (count < 3U || count > 32U) {
    return false;
  }
  for (const auto &point : polygon) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
      return false;
    }
  }
  const long double twice_area = twiceSignedArea(polygon);
  if (!std::isfinite(twice_area) || std::abs(twice_area) <= 2.0e-8L ||
      std::abs(twice_area) > 800.0L) {
    return false;
  }
  const long double winding = twice_area > 0.0L ? 1.0L : -1.0L;
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t next = (i + 1U) % count;
    const auto &a = polygon[i];
    const auto &b = polygon[next];
    const long double length = edgeLength(a, b);
    if (!std::isfinite(length) || length <= kBoundaryEpsilonM) {
      return false;
    }
    // HH_261007 - Every other vertex must lie strictly inside every oriented
    // edge. Unlike adjacent-turn tests, this also rejects self-crossing stars.
    for (std::size_t j = 0; j < count; ++j) {
      if (j == i || j == next) {
        continue;
      }
      const long double side = winding * cross(
          a, b, polygon[j].first, polygon[j].second);
      if (!std::isfinite(side) || side <= kBoundaryEpsilonM * length) {
        return false;
      }
    }
  }
  return true;
}

inline bool manualServicePolygonContains(const ManualServicePolygon &polygon,
                                         const double x, const double y) {
  using namespace manual_service_area_detail;
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !validManualServicePolygon(polygon)) {
    return false;
  }
  const long double winding = twiceSignedArea(polygon) > 0.0L ? 1.0L : -1.0L;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i];
    const auto &b = polygon[(i + 1U) % polygon.size()];
    const long double side = winding * cross(a, b, x, y);
    // HH_261007 - Include the authored boundary with only 1e-8 m numerical
    // tolerance; this is not an operational clearance or centroid fallback.
    if (!std::isfinite(side) || side < -kBoundaryEpsilonM * edgeLength(a, b)) {
      return false;
    }
  }
  return true;
}

inline bool manualServicePolygonsTouch(const ManualServicePolygon &first,
                                      const ManualServicePolygon &second) {
  using namespace manual_service_area_detail;
  if (!validManualServicePolygon(first) || !validManualServicePolygon(second))
    return false;
  // Convex separating-edge test includes shared boundaries, including an
  // access edge touching the site without overlapping its interior.
  const auto separated = [](const auto &a, const auto &b) {
    const long double winding = twiceSignedArea(a) > 0.0L ? 1.0L : -1.0L;
    for (std::size_t i = 0; i < a.size(); ++i) {
      const auto &start = a[i];
      const auto &end = a[(i + 1U) % a.size()];
      bool all_outside = true;
      for (const auto &point : b) {
        if (winding * cross(start, end, point.first, point.second) >=
            -kBoundaryEpsilonM * edgeLength(start, end)) {
          all_outside = false;
          break;
        }
      }
      if (all_outside) return true;
    }
    return false;
  };
  return !separated(first, second) && !separated(second, first);
}

inline bool validManualServiceArea(const ManualServiceArea &area) {
  if (!validManualServicePolygon(area.polygon)) return false;
  for (const auto &access : area.access_polygons) {
    if (!validManualServicePolygon(access) ||
        !manualServicePolygonsTouch(area.polygon, access)) return false;
  }
  return true;
}

inline bool manualServiceAreaContains(const ManualServiceArea &area,
                                      const double x, const double y) {
  if (!validManualServiceArea(area)) return false;
  if (manualServicePolygonContains(area.polygon, x, y)) return true;
  for (const auto &access : area.access_polygons)
    if (manualServicePolygonContains(access, x, y)) return true;
  return false;
}

inline bool manualServiceAreasOverlap(const ManualServiceArea &first,
                                     const ManualServiceArea &second) {
  if (manualServicePolygonsTouch(first.polygon, second.polygon)) return true;
  for (const auto &access : first.access_polygons) {
    if (manualServicePolygonsTouch(access, second.polygon)) return true;
    for (const auto &other : second.access_polygons)
      if (manualServicePolygonsTouch(access, other)) return true;
  }
  for (const auto &access : second.access_polygons)
    if (manualServicePolygonsTouch(first.polygon, access)) return true;
  return false;
}

}  // namespace camrod_control
