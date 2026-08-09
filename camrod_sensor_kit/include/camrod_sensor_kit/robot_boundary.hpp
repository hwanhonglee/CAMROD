#pragma once

// HH_260809 - Generate one tapered, rounded robot boundary for visualization,
// Nav2, and final lanelet safety instead of maintaining independent rectangles.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace camrod
{

struct PlanarBoundaryPoint
{
  double x{0.0};
  double y{0.0};
};

struct RobotBoundaryExtents
{
  double front{0.0};
  double rear{0.0};
  double left{0.0};
  double right{0.0};
};

struct RobotBoundaryMargins
{
  double front{0.0};
  double rear{0.0};
  double left{0.0};
  double right{0.0};
};

struct RobotBoundaryShape
{
  RobotBoundaryExtents extents;
  double front_taper{0.12};
  double front_shoulder_depth{0.12};
  double corner_radius{0.05};
  int corner_samples{4};
};

namespace boundary_detail
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

inline PlanarBoundaryPoint add(
  const PlanarBoundaryPoint & lhs, const PlanarBoundaryPoint & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

inline PlanarBoundaryPoint subtract(
  const PlanarBoundaryPoint & lhs, const PlanarBoundaryPoint & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

inline PlanarBoundaryPoint multiply(const PlanarBoundaryPoint & point, const double scale)
{
  return {point.x * scale, point.y * scale};
}

inline double dot(const PlanarBoundaryPoint & lhs, const PlanarBoundaryPoint & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline double cross(const PlanarBoundaryPoint & lhs, const PlanarBoundaryPoint & rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

inline double norm(const PlanarBoundaryPoint & point)
{
  return std::hypot(point.x, point.y);
}

inline PlanarBoundaryPoint normalized(const PlanarBoundaryPoint & point)
{
  const double length = norm(point);
  if (length <= kEpsilon) {
    return {};
  }
  return {point.x / length, point.y / length};
}

inline double signedArea(const std::vector<PlanarBoundaryPoint> & polygon)
{
  if (polygon.size() < 3) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & current = polygon[index];
    const auto & next = polygon[(index + 1) % polygon.size()];
    twice_area += cross(current, next);
  }
  return 0.5 * twice_area;
}

inline std::vector<PlanarBoundaryPoint> makeSharpBoundary(
  const RobotBoundaryShape & shape)
{
  const double front = std::max(0.0, shape.extents.front);
  const double rear = std::max(0.0, shape.extents.rear);
  const double left = std::max(0.0, shape.extents.left);
  const double right = std::max(0.0, shape.extents.right);
  const double total_width = left + right;
  const double total_length = front + rear;
  if (total_width <= kEpsilon || total_length <= kEpsilon) {
    return {};
  }

  const double taper = std::clamp(shape.front_taper, 0.0, 0.45 * total_width);
  const double shoulder_depth = std::clamp(
    shape.front_shoulder_depth, 0.0, 0.45 * total_length);
  if (taper <= kEpsilon || shoulder_depth <= kEpsilon) {
    return {
      {front, left},
      {front, -right},
      {-rear, -right},
      {-rear, left}};
  }

  // Clockwise order: short front face, right shoulder/side, rear, left side/shoulder.
  return {
    {front, left - taper},
    {front, -right + taper},
    {front - shoulder_depth, -right},
    {-rear, -right},
    {-rear, left},
    {front - shoulder_depth, left}};
}

inline std::vector<PlanarBoundaryPoint> roundConvexPolygon(
  const std::vector<PlanarBoundaryPoint> & sharp_polygon,
  const double requested_radius,
  const int requested_samples)
{
  if (sharp_polygon.size() < 3 || requested_radius <= kEpsilon) {
    return sharp_polygon;
  }

  const bool clockwise = signedArea(sharp_polygon) < 0.0;
  const int samples = std::max(1, requested_samples);
  std::vector<PlanarBoundaryPoint> rounded;
  rounded.reserve(sharp_polygon.size() * static_cast<std::size_t>(samples + 1));

  for (std::size_t index = 0; index < sharp_polygon.size(); ++index) {
    const auto & previous = sharp_polygon[
      (index + sharp_polygon.size() - 1) % sharp_polygon.size()];
    const auto & vertex = sharp_polygon[index];
    const auto & next = sharp_polygon[(index + 1) % sharp_polygon.size()];
    const auto toward_previous = subtract(previous, vertex);
    const auto toward_next = subtract(next, vertex);
    const double previous_length = norm(toward_previous);
    const double next_length = norm(toward_next);
    if (previous_length <= kEpsilon || next_length <= kEpsilon) {
      rounded.push_back(vertex);
      continue;
    }

    const auto previous_unit = multiply(toward_previous, 1.0 / previous_length);
    const auto next_unit = multiply(toward_next, 1.0 / next_length);
    const double interior_angle = std::acos(std::clamp(
        dot(previous_unit, next_unit), -1.0, 1.0));
    const double half_angle = 0.5 * interior_angle;
    const double tangent_denominator = std::tan(half_angle);
    if (half_angle <= kEpsilon || std::abs(tangent_denominator) <= kEpsilon) {
      rounded.push_back(vertex);
      continue;
    }

    double tangent_distance = requested_radius / tangent_denominator;
    tangent_distance = std::min(
      tangent_distance, 0.49 * std::min(previous_length, next_length));
    const double effective_radius = tangent_distance * tangent_denominator;
    const auto tangent_start = add(vertex, multiply(previous_unit, tangent_distance));
    const auto tangent_end = add(vertex, multiply(next_unit, tangent_distance));
    const auto bisector = normalized(add(previous_unit, next_unit));
    const double center_distance = effective_radius / std::sin(half_angle);
    const auto center = add(vertex, multiply(bisector, center_distance));

    const auto start_vector = subtract(tangent_start, center);
    const auto end_vector = subtract(tangent_end, center);
    const double start_angle = std::atan2(start_vector.y, start_vector.x);
    double arc_angle = std::atan2(cross(start_vector, end_vector), dot(start_vector, end_vector));
    if (clockwise && arc_angle > 0.0) {
      arc_angle -= 2.0 * kPi;
    } else if (!clockwise && arc_angle < 0.0) {
      arc_angle += 2.0 * kPi;
    }

    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const double angle = start_angle + ratio * arc_angle;
      rounded.push_back({
        center.x + effective_radius * std::cos(angle),
        center.y + effective_radius * std::sin(angle)});
    }
  }
  return rounded;
}

struct OffsetLine
{
  PlanarBoundaryPoint point;
  PlanarBoundaryPoint direction;
};

inline PlanarBoundaryPoint intersectLines(
  const OffsetLine & first,
  const OffsetLine & second,
  const PlanarBoundaryPoint & fallback)
{
  const double denominator = cross(first.direction, second.direction);
  if (std::abs(denominator) <= kEpsilon) {
    return fallback;
  }
  const double distance = cross(
    subtract(second.point, first.point), second.direction) / denominator;
  return add(first.point, multiply(first.direction, distance));
}

inline double directionalMargin(
  const PlanarBoundaryPoint & outward_normal,
  const RobotBoundaryMargins & margins)
{
  const double longitudinal = outward_normal.x >= 0.0 ? margins.front : margins.rear;
  const double lateral = outward_normal.y >= 0.0 ? margins.left : margins.right;
  return std::hypot(
    outward_normal.x * std::max(0.0, longitudinal),
    outward_normal.y * std::max(0.0, lateral));
}

inline std::vector<PlanarBoundaryPoint> offsetConvexPolygon(
  const std::vector<PlanarBoundaryPoint> & polygon,
  const RobotBoundaryMargins & margins)
{
  if (polygon.size() < 3) {
    return polygon;
  }
  const bool clockwise = signedArea(polygon) < 0.0;
  std::vector<OffsetLine> shifted_edges;
  shifted_edges.reserve(polygon.size());
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto direction = subtract(polygon[(index + 1) % polygon.size()], polygon[index]);
    const double edge_length = norm(direction);
    if (edge_length <= kEpsilon) {
      return polygon;
    }
    const auto unit = multiply(direction, 1.0 / edge_length);
    const PlanarBoundaryPoint outward = clockwise ?
      PlanarBoundaryPoint{-unit.y, unit.x} : PlanarBoundaryPoint{unit.y, -unit.x};
    const double margin = directionalMargin(outward, margins);
    shifted_edges.push_back({add(polygon[index], multiply(outward, margin)), direction});
  }

  std::vector<PlanarBoundaryPoint> expanded;
  expanded.reserve(polygon.size());
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto & previous_edge = shifted_edges[
      (index + shifted_edges.size() - 1) % shifted_edges.size()];
    const auto & current_edge = shifted_edges[index];
    expanded.push_back(intersectLines(previous_edge, current_edge, polygon[index]));
  }
  return expanded;
}

}  // namespace boundary_detail

inline std::vector<PlanarBoundaryPoint> makeRobotBoundary(const RobotBoundaryShape & shape)
{
  return boundary_detail::roundConvexPolygon(
    boundary_detail::makeSharpBoundary(shape),
    std::max(0.0, shape.corner_radius),
    shape.corner_samples);
}

inline std::vector<PlanarBoundaryPoint> makeExpandedRobotBoundary(
  const RobotBoundaryShape & body_shape,
  const RobotBoundaryMargins & margins)
{
  const auto sharp_body = boundary_detail::makeSharpBoundary(body_shape);
  const auto expanded_sharp = boundary_detail::offsetConvexPolygon(sharp_body, margins);
  const double minimum_margin = std::max(
    0.0,
    std::min({margins.front, margins.rear, margins.left, margins.right}));
  return boundary_detail::roundConvexPolygon(
    expanded_sharp,
    std::max(0.0, body_shape.corner_radius) + minimum_margin,
    body_shape.corner_samples);
}

inline std::vector<std::pair<double, double>> asBoundaryPairs(
  const std::vector<PlanarBoundaryPoint> & boundary)
{
  std::vector<std::pair<double, double>> pairs;
  pairs.reserve(boundary.size());
  for (const auto & point : boundary) {
    pairs.emplace_back(point.x, point.y);
  }
  return pairs;
}

}  // namespace camrod
