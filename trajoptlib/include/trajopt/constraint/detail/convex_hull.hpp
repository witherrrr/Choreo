// Copyright (c) TrajoptLib contributors

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "trajopt/geometry/translation2.hpp"

namespace trajopt::detail {

/// Returns the convex hull of the given points in counterclockwise order using
/// Andrew's monotone chain algorithm. Duplicate points and points on the
/// interior of hull edges are dropped.
///
/// @param points The points.
/// @return The convex hull's vertices in counterclockwise order.
inline std::vector<Translation2d> convex_hull(
    std::vector<Translation2d> points) {
  std::ranges::sort(points, [](const Translation2d& a, const Translation2d& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  points.erase(std::ranges::unique(points).begin(), points.end());
  if (points.size() < 3) {
    return points;
  }

  auto cross = [](const Translation2d& o, const Translation2d& a,
                  const Translation2d& b) { return (a - o).cross(b - o); };

  std::vector<Translation2d> hull(2 * points.size());
  size_t k = 0;

  // Lower hull
  for (size_t i = 0; i < points.size(); ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], points[i]) <= 0.0) {
      --k;
    }
    hull[k++] = points[i];
  }

  // Upper hull
  for (size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(hull[k - 2], hull[k - 1], points[i - 1]) <= 0.0) {
      --k;
    }
    hull[k++] = points[i - 1];
  }

  // The last point is the same as the first
  hull.resize(k - 1);
  return hull;
}

/// Returns a unit vector normal to a line separating the convex hull of the
/// given points from a point.
///
/// If the point is outside the hull, this is the direction from the closest
/// point on the hull to the point. If the point is inside the hull, this is the
/// direction from the point to the closest point on the hull's boundary, which
/// is the direction the hull would have to move the least to no longer contain
/// the point.
///
/// @param points The points whose convex hull to separate from the point.
/// @param point The point.
/// @return The separating direction, or nullopt if the point is on the hull's
///     boundary or the points are empty.
inline std::optional<Translation2d> separating_direction(
    const std::vector<Translation2d>& points, const Translation2d& point) {
  auto hull = convex_hull(points);
  if (hull.empty()) {
    return std::nullopt;
  }

  // A hull with fewer than three vertices has no interior
  bool inside = hull.size() >= 3;

  Translation2d closest = hull.front();
  double closest_squared_distance = INFINITY;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];

    // Closest point on edge ab to the point
    auto ab = b - a;
    double t = 0.0;
    if (double length_squared = ab.squared_norm(); length_squared > 0.0) {
      t = std::clamp((point - a).dot(ab) / length_squared, 0.0, 1.0);
    }
    auto q = a + ab * t;

    double squared_distance = (point - q).squared_norm();
    if (squared_distance < closest_squared_distance) {
      closest_squared_distance = squared_distance;
      closest = q;
    }

    // The hull is counterclockwise, so points inside it are to the left of
    // every edge
    if (ab.cross(point - a) < 0.0) {
      inside = false;
    }
  }

  if (!(closest_squared_distance > 0.0) ||
      !std::isfinite(closest_squared_distance)) {
    return std::nullopt;
  }

  double distance = std::sqrt(closest_squared_distance);
  if (inside) {
    return (closest - point) / distance;
  } else {
    return (point - closest) / distance;
  }
}

}  // namespace trajopt::detail
