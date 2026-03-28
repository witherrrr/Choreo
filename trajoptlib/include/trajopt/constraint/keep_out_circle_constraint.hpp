// Copyright (c) TrajoptLib contributors

#pragma once

#include <cassert>
#include <cmath>
#include <span>
#include <utility>
#include <vector>

#include <sleipnir/autodiff/variable.hpp>
#include <sleipnir/optimization/problem.hpp>

#include "trajopt/constraint/detail/convex_hull.hpp"
#include "trajopt/geometry/pose2.hpp"
#include "trajopt/geometry/rotation2.hpp"
#include "trajopt/geometry/translation2.hpp"
#include "trajopt/util/symbol_exports.hpp"

namespace trajopt {

/// Keep-out circle constraint.
///
/// Specifies the required minimum distance between a convex polygon on the
/// robot's frame (e.g., the bumpers) and a point on the field. In other words,
/// the polygon must stay out of a circle on the field.
///
/// A convex polygon and a circle are at least r apart iff there's a line with
/// the polygon on one side and the circle on the other. With n the line's unit
/// normal (pointing from the polygon toward the circle), p the circle's center,
/// and vⱼ the polygon's vertices, that's
///
///   n · (p − vⱼ) ≥ r  for all j
///   ‖n‖₂ = 1
///
/// The unit normal is an extra decision variable, so this formulation is exact
/// for convex polygons and smooth. Unlike constraining the distance from each
/// vertex and edge to the circle, it also rejects poses where the circle is
/// inside the polygon.
///
/// When applied to the continuum between two consecutive samples, the
/// separating line is required to separate the circle from the convex hull of
/// the polygon at both samples, which is the area the polygon sweeps between
/// them (exactly for pure translation, approximately for small rotations). This
/// prevents the robot from tunneling through the circle between samples.
///
/// If the polygon isn't convex, the constraint applies to its convex hull.
class TRAJOPT_DLLEXPORT KeepOutCircleConstraint {
 public:
  /// Constructs a KeepOutCircleConstraint.
  ///
  /// @param robot_polygon Vertices of a convex polygon on the robot's frame.
  ///     Must have at least one vertex.
  /// @param field_point Center of the keep-out circle on the field.
  /// @param min_distance Minimum distance between the robot polygon and the
  ///     field point (the keep-out circle's radius). Must be nonnegative.
  explicit KeepOutCircleConstraint(std::vector<Translation2d> robot_polygon,
                                   Translation2d field_point,
                                   double min_distance)
      : m_robot_polygon{std::move(robot_polygon)},
        m_field_point{std::move(field_point)},
        m_min_distance{min_distance} {
    assert(!m_robot_polygon.empty());
    assert(min_distance >= 0.0);
  }

  /// Applies this constraint to the given problem at a single sample.
  ///
  /// @param problem The optimization problem.
  /// @param pose The robot's pose.
  /// @param linear_velocity The robot's linear velocity.
  /// @param angular_velocity The robot's angular velocity.
  /// @param linear_acceleration The robot's linear acceleration.
  /// @param angular_acceleration The robot's angular acceleration.
  void apply(
      slp::Problem<double>& problem, const Pose2v<double>& pose,
      [[maybe_unused]] const Translation2v<double>& linear_velocity,
      [[maybe_unused]] const slp::Variable<double>& angular_velocity,
      [[maybe_unused]] const Translation2v<double>& linear_acceleration,
      [[maybe_unused]] const slp::Variable<double>& angular_acceleration) {
    const Pose2v<double>* poses[] = {&pose};
    apply_separating_line(problem, poses);
  }

  /// Applies this constraint to the given problem over the continuum between
  /// two consecutive samples.
  ///
  /// The constraint is applied to the convex hull of the robot polygon at both
  /// poses, so the robot can't pass through the keep-out circle between the
  /// samples.
  ///
  /// @param problem The optimization problem.
  /// @param start_pose The robot's pose at the start of the interval.
  /// @param end_pose The robot's pose at the end of the interval.
  void apply_swept(slp::Problem<double>& problem,
                   const Pose2v<double>& start_pose,
                   const Pose2v<double>& end_pose) {
    const Pose2v<double>* poses[] = {&start_pose, &end_pose};
    apply_separating_line(problem, poses);
  }

 private:
  std::vector<Translation2d> m_robot_polygon;
  Translation2d m_field_point;
  double m_min_distance;

  /// Requires a line to separate the field point by at least the minimum
  /// distance from the robot polygon at all of the given poses.
  ///
  /// @param problem The optimization problem.
  /// @param poses The robot's poses.
  void apply_separating_line(slp::Problem<double>& problem,
                             std::span<const Pose2v<double>* const> poses) {
    // The line's unit normal is a decision variable. Its initial guess is the
    // normal of the line separating the field point from the convex hull of
    // the polygon at the poses' initial guesses. If the field point is inside
    // the hull, the initial guess is the direction the hull has to move the
    // least to get out of the way, so the solver only has to move the robot
    // sideways instead of also rotating the line.
    std::vector<Translation2d> vertices;
    for (const auto* pose : poses) {
      // Variable::value() isn't const, so evaluate copies
      slp::Variable<double> x = pose->x();
      slp::Variable<double> y = pose->y();
      slp::Variable<double> cos = pose->rotation().cos();
      slp::Variable<double> sin = pose->rotation().sin();
      Pose2d pose_guess{x.value(), y.value(),
                        Rotation2d{cos.value(), sin.value()}};
      for (const auto& robot_point : m_robot_polygon) {
        vertices.push_back(pose_guess.translation() +
                           robot_point.rotate_by(pose_guess.rotation()));
      }
    }
    Translation2d n_guess{1.0, 0.0};
    if (auto direction = detail::separating_direction(vertices, m_field_point);
        direction && std::isfinite(direction->x()) &&
        std::isfinite(direction->y())) {
      n_guess = *direction;
    }

    slp::Variable<double> n_x = problem.decision_variable();
    slp::Variable<double> n_y = problem.decision_variable();
    n_x.set_value(n_guess.x());
    n_y.set_value(n_guess.y());
    Translation2v<double> n{n_x, n_y};

    // ‖n‖₂ = 1
    problem.subject_to(n.squared_norm() == 1.0);

    // n · (p − vⱼ) ≥ r
    for (const auto* pose : poses) {
      for (const auto& robot_point : m_robot_polygon) {
        auto vertex =
            pose->translation() + robot_point.rotate_by(pose->rotation());
        problem.subject_to(n.dot(m_field_point - vertex) >= m_min_distance);
      }
    }
  }
};

}  // namespace trajopt
