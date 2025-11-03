#ifndef NAVIGATION__TYPES_HPP_
#define NAVIGATION__TYPES_HPP_

#include <cmath>
#include <vector>

namespace navigation
{

/**
 * @brief 2D pose representation (x, y, theta)
 */
struct Pose2D
{
  double x;
  double y;
  double theta;

  Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  Pose2D(double x_in, double y_in, double theta_in) : x(x_in), y(y_in), theta(theta_in) {}

  /**
   * @brief Compute distance to another pose
   */
  double distanceTo(const Pose2D & other) const
  {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }
};

/**
 * @brief Trajectory point with pose, velocities, timestamp, and curvature
 */
struct TrajectoryPoint
{
  Pose2D pose;          // Position and orientation
  double v;             // Linear velocity (m/s)
  double omega;         // Angular velocity (rad/s)
  double t;             // Timestamp (seconds from start)
  double curvature;     // Path curvature (1/m)
  double arc_length;    // Cumulative arc length along path

  TrajectoryPoint()
  : v(0.0), omega(0.0), t(0.0), curvature(0.0), arc_length(0.0)
  {}
};

/**
 * @brief Utility functions for path calculations
 */
namespace utils
{

/**
 * @brief Compute distance between two poses
 */
inline double distance(const Pose2D & p1, const Pose2D & p2)
{
  return p1.distanceTo(p2);
}

/**
 * @brief Linear interpolation between two poses
 */
inline Pose2D lerp(const Pose2D & p1, const Pose2D & p2, double alpha)
{
  Pose2D result;
  result.x = p1.x + alpha * (p2.x - p1.x);
  result.y = p1.y + alpha * (p2.y - p1.y);
  result.theta = p1.theta + alpha * Pose2D::normalizeAngle(p2.theta - p1.theta);
  return result;
}

/**
 * @brief Compute orientation from two points
 */
inline double computeOrientation(const Pose2D & from, const Pose2D & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

}  // namespace utils

}  // namespace navigation

#endif  // NAVIGATION__TYPES_HPP_

