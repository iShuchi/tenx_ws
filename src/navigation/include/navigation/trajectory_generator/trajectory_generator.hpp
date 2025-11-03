#ifndef NAVIGATION__TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_
#define NAVIGATION__TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include "navigation/types.hpp"

namespace navigation
{

/**
 * @brief Trajectory generator with time parameterization and velocity profiles
 */
class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  ~TrajectoryGenerator() = default;

  /**
   * @brief Generate trajectory from smoothed path
   * @param smoothed_path Input smoothed path
   * @return Generated trajectory with velocities and timestamps
   */
  std::vector<TrajectoryPoint> generateTrajectory(const std::vector<Pose2D> & smoothed_path);

  /**
   * @brief Configure trajectory generator parameters
   */
  void configure(
    double resolution,
    double v_max,
    double a_max,
    bool curvature_velocity_scaling);

  /**
   * @brief Compute curvature at a point along the path
   */
  static double computeCurvature(
    const std::vector<Pose2D> & path,
    size_t idx);

private:
  double resolution_;              // Sampling resolution (meters)
  double v_max_;                  // Maximum velocity (m/s)
  double a_max_;                  // Maximum acceleration (m/s^2)
  bool curvature_velocity_scaling_; // Enable curvature-based velocity scaling

  /**
   * @brief Compute arc length between two points
   */
  double computeArcLength(const Pose2D & p1, const Pose2D & p2) const;

  /**
   * @brief Compute cumulative arc lengths along path
   */
  std::vector<double> computeArcLengths(const std::vector<Pose2D> & path) const;

  /**
   * @brief Generate velocity profile with trapezoidal acceleration
   */
  double computeVelocityForCurvature(double curvature) const;
};

}  // namespace navigation

#endif  // NAVIGATION__TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_HPP_

