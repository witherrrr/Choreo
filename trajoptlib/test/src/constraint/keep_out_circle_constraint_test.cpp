// Copyright (c) TrajoptLib contributors

#include <algorithm>
#include <cmath>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sleipnir/optimization/problem.hpp>
#include <sleipnir/optimization/solver/exit_status.hpp>
#include <trajopt/constraint/keep_out_circle_constraint.hpp>
#include <trajopt/differential_trajectory_generator.hpp>
#include <trajopt/geometry/pose2.hpp>
#include <trajopt/geometry/translation2.hpp>
#include <trajopt/swerve_trajectory_generator.hpp>

using Catch::Matchers::WithinAbs;

namespace {

/// Returns the distance from a point to the convex hull of a set of points, or
/// zero if the point is inside the hull.
double distance_to_convex_hull(std::vector<trajopt::Translation2d> points,
                               const trajopt::Translation2d& point) {
  auto cross =
      [](const trajopt::Translation2d& o, const trajopt::Translation2d& a,
         const trajopt::Translation2d& b) { return (a - o).cross(b - o); };

  // Andrew's monotone chain algorithm produces a counterclockwise hull
  std::ranges::sort(points, [](const auto& a, const auto& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  std::vector<trajopt::Translation2d> hull(2 * points.size());
  size_t k = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], points[i]) <= 0.0) {
      --k;
    }
    hull[k++] = points[i];
  }
  for (size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(hull[k - 2], hull[k - 1], points[i - 1]) <= 0.0) {
      --k;
    }
    hull[k++] = points[i - 1];
  }
  hull.resize(k - 1);

  bool inside = hull.size() >= 3;
  double min_edge_distance = INFINITY;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& a = hull[i];
    const auto& b = hull[(i + 1) % hull.size()];

    if (cross(a, b, point) < 0.0) {
      inside = false;
    }

    auto ab = b - a;
    double t = 0.0;
    if (ab.squared_norm() > 0.0) {
      t = std::clamp((point - a).dot(ab) / ab.squared_norm(), 0.0, 1.0);
    }
    min_edge_distance =
        std::min(min_edge_distance, (a + ab * t).distance(point));
  }

  return inside ? 0.0 : min_edge_distance;
}

/// Returns the robot polygon's vertices on the field for the given pose.
std::vector<trajopt::Translation2d> field_vertices(
    const std::vector<trajopt::Translation2d>& robot_polygon, double x,
    double y, double heading) {
  std::vector<trajopt::Translation2d> vertices;
  for (const auto& robot_point : robot_polygon) {
    vertices.push_back(trajopt::Translation2d{x, y} +
                       robot_point.rotate_by(trajopt::Rotation2d{heading}));
  }
  return vertices;
}

/// Checks that the area swept by the robot polygon between every pair of
/// consecutive samples stays at least min_distance away from the field point,
/// and that the trajectory actually deviated from the straight-line initial
/// guess along y = 0.
void check_no_tunneling(
    const std::vector<trajopt::Translation2d>& robot_polygon,
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& heading,
    const trajopt::Translation2d& field_point, double min_distance) {
  REQUIRE(x.size() == y.size());
  REQUIRE(x.size() == heading.size());
  REQUIRE(x.size() >= 2);

  double max_abs_y = 0.0;
  for (size_t k = 0; k + 1 < x.size(); ++k) {
    auto vertices = field_vertices(robot_polygon, x[k], y[k], heading[k]);
    auto next_vertices =
        field_vertices(robot_polygon, x[k + 1], y[k + 1], heading[k + 1]);
    vertices.insert(vertices.end(), next_vertices.begin(), next_vertices.end());

    INFO("Sample " << k);
    CHECK(distance_to_convex_hull(vertices, field_point) >=
          min_distance - 1e-3);

    max_abs_y = std::max({max_abs_y, std::abs(y[k]), std::abs(y[k + 1])});
  }
  CHECK(max_abs_y > 0.4);
}

const std::vector<trajopt::Translation2d> square_bumpers{
    {+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}};

}  // namespace

TEST_CASE("KeepOutCircleConstraint - Circle inside polygon is infeasible",
          "[KeepOutCircleConstraint]") {
  // Get a 1 m × 1 m robot as close as possible to a keep-out circle of radius
  // 0.25 m at the origin. The old distance-to-vertices-and-edges formulation
  // was satisfied with the robot centered on the circle. The closest the robot
  // center can actually get is 0.5 m (half width) + 0.25 m (radius).
  slp::Problem<double> problem;
  auto x = problem.decision_variable();
  auto y = problem.decision_variable();
  x.set_value(2.0);
  y.set_value(0.0);
  trajopt::Pose2v<double> pose{
      x, y,
      trajopt::Rotation2v<double>{slp::Variable<double>{1.0},
                                  slp::Variable<double>{0.0}}};

  trajopt::KeepOutCircleConstraint constraint{square_bumpers, {0.0, 0.0}, 0.25};
  constraint.apply(problem, pose, trajopt::Translation2v<double>{0.0, 0.0},
                   slp::Variable<double>{0.0},
                   trajopt::Translation2v<double>{0.0, 0.0},
                   slp::Variable<double>{0.0});

  problem.minimize(x * x + y * y);

  CHECK(problem.solve({.tolerance = 1e-8}) == slp::ExitStatus::SUCCESS);
  CHECK_THAT(std::hypot(x.value(), y.value()), WithinAbs(0.75, 1e-4));
  CHECK(distance_to_convex_hull(
            field_vertices(square_bumpers, x.value(), y.value(), 0.0),
            {0.0, 0.0}) >= 0.25 - 1e-4);
}

TEST_CASE("KeepOutCircleConstraint - Swept area avoids circle",
          "[KeepOutCircleConstraint]") {
  // A 1 m × 1 m robot moves from (-1, 0) to (1, y₁) in one step. A keep-out
  // circle of radius 0.1 m at (0, 0.3) sits between the samples, so a
  // per-sample constraint would allow y₁ = 0 and let the robot tunnel through
  // the circle. Find the smallest |y₁| that avoids the circle.
  slp::Problem<double> problem;
  auto y_1 = problem.decision_variable();
  y_1.set_value(0.0);

  trajopt::Rotation2v<double> heading{slp::Variable<double>{1.0},
                                      slp::Variable<double>{0.0}};
  trajopt::Pose2v<double> start_pose{slp::Variable<double>{-1.0},
                                     slp::Variable<double>{0.0}, heading};
  trajopt::Pose2v<double> end_pose{slp::Variable<double>{1.0}, y_1, heading};

  trajopt::KeepOutCircleConstraint constraint{square_bumpers, {0.0, 0.3}, 0.1};
  constraint.apply_swept(problem, start_pose, end_pose);

  problem.minimize(y_1 * y_1);

  CHECK(problem.solve({.tolerance = 1e-8}) == slp::ExitStatus::SUCCESS);

  // The robot must dip below the circle. The closest hull edge runs from the
  // start pose's top-right corner (-0.5, 0.5) to the end pose's top-right
  // corner (1.5, y₁ + 0.5). Solving for the circle center being 0.1 m above
  // that edge gives 0.24y₁² + 0.4y₁ + 0.12 = 0 with y₁ < -0.8.
  const double expected_y_1 = -(0.4 + std::sqrt(0.16 - 4 * 0.24 * 0.12)) / 0.48;
  CHECK_THAT(y_1.value(), WithinAbs(expected_y_1, 1e-4));

  auto vertices = field_vertices(square_bumpers, -1.0, 0.0, 0.0);
  auto end_vertices = field_vertices(square_bumpers, 1.0, y_1.value(), 0.0);
  vertices.insert(vertices.end(), end_vertices.begin(), end_vertices.end());
  CHECK_THAT(distance_to_convex_hull(vertices, {0.0, 0.3}),
             WithinAbs(0.1, 1e-4));
}

TEST_CASE("KeepOutCircleConstraint - Swerve trajectory doesn't tunnel",
          "[KeepOutCircleConstraint]") {
  trajopt::SwerveDrivetrain swerve_drivetrain{
      .mass = 45,
      .moi = 6,
      .wheel_radius = 0.04,
      .wheel_max_angular_velocity = 70,
      .wheel_max_torque = 2,
      .wheel_cof = 1.5,
      .modules = {{+0.3, +0.3}, {+0.3, -0.3}, {-0.3, +0.3}, {-0.3, -0.3}}};

  // The straight-line initial guess drives right through the circle
  const trajopt::Translation2d field_point{1.5, 0.1};
  constexpr double radius = 0.15;

  trajopt::SwervePathBuilder path;
  path.set_drivetrain(swerve_drivetrain);
  path.set_bumpers(0.5, 0.5, 0.5, 0.5);
  path.pose_wpt(0, 0.0, 0.0, 0.0);
  path.pose_wpt(1, 3.0, 0.0, 0.0);
  path.wpt_constraint(0, trajopt::LinearVelocityMaxMagnitudeConstraint{0.0});
  path.wpt_constraint(1, trajopt::LinearVelocityMaxMagnitudeConstraint{0.0});
  path.sgmt_constraint(
      0, 1,
      trajopt::KeepOutCircleConstraint{path.get_bumpers().at(0).points,
                                       field_point, radius});
  path.set_control_interval_counts({30});

  trajopt::SwerveTrajectoryGenerator generator{path};
  auto solution = generator.generate();
  REQUIRE(solution.has_value());

  std::vector<double> heading;
  for (size_t k = 0; k < solution->x.size(); ++k) {
    heading.push_back(std::atan2(solution->thetasin[k], solution->thetacos[k]));
  }
  check_no_tunneling(path.get_bumpers().at(0).points, solution->x, solution->y,
                     heading, field_point, radius);
}

TEST_CASE("KeepOutCircleConstraint - Differential trajectory doesn't tunnel",
          "[KeepOutCircleConstraint]") {
  trajopt::DifferentialDrivetrain differential_drivetrain{
      .mass = 45,
      .moi = 6,
      .wheel_radius = 0.08,
      .wheel_max_angular_velocity = 70,
      .wheel_max_torque = 5,
      .wheel_cof = 1.5,
      .trackwidth = 0.6};

  // The straight-line initial guess drives right through the circle
  const trajopt::Translation2d field_point{1.5, 0.1};
  constexpr double radius = 0.15;

  trajopt::DifferentialPathBuilder path;
  path.set_drivetrain(differential_drivetrain);
  path.set_bumpers(0.5, 0.5, 0.5, 0.5);
  path.pose_wpt(0, 0.0, 0.0, 0.0);
  path.pose_wpt(1, 3.0, 0.0, 0.0);
  path.wpt_constraint(0, trajopt::LinearVelocityMaxMagnitudeConstraint{0.0});
  path.wpt_constraint(1, trajopt::LinearVelocityMaxMagnitudeConstraint{0.0});
  path.sgmt_constraint(
      0, 1,
      trajopt::KeepOutCircleConstraint{path.get_bumpers().at(0).points,
                                       field_point, radius});
  path.set_control_interval_counts({30});

  trajopt::DifferentialTrajectoryGenerator generator{path};
  auto solution = generator.generate();
  REQUIRE(solution.has_value());

  check_no_tunneling(path.get_bumpers().at(0).points, solution->x, solution->y,
                     solution->heading, field_point, radius);
}
