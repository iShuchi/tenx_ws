#include "navigation/trajectory_generator/trajectory_generator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation
{

TrajectoryGenerator::TrajectoryGenerator()
: resolution_(0.05),
  v_max_(0.5),
  a_max_(0.5),
  curvature_velocity_scaling_(true)
{
}

void TrajectoryGenerator::configure(
  double resolution,
  double v_max,
  double a_max,
  bool curvature_velocity_scaling)
{
  resolution_ = resolution;
  v_max_ = v_max;
  a_max_ = a_max;
  curvature_velocity_scaling_ = curvature_velocity_scaling;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(
  const std::vector<Pose2D> & smoothed_path)
{
  std::vector<TrajectoryPoint> trajectory;

  if (smoothed_path.size() < 2) {
    return trajectory;
  }

  // Compute arc lengths
  std::vector<double> arc_lengths = computeArcLengths(smoothed_path);
  double total_length = arc_lengths.back();

  // Compute curvatures at each point
  std::vector<double> curvatures(smoothed_path.size());
  for (size_t i = 0; i < smoothed_path.size(); ++i) {
    curvatures[i] = computeCurvature(smoothed_path, i);
  }

  // Generate trajectory points along path
  double current_s = 0.0;
  double current_t = 0.0;
  double current_v = 0.0;

  while (current_s < total_length) {
    // Find segment containing current_s
    size_t segment_idx = 0;
    for (size_t i = 1; i < arc_lengths.size(); ++i) {
      if (arc_lengths[i] >= current_s) {
        segment_idx = i - 1;
        break;
      }
    }
    if (segment_idx >= smoothed_path.size() - 1) {
      segment_idx = smoothed_path.size() - 2;
    }

    // Interpolate position along segment
    double s_start = (segment_idx > 0) ? arc_lengths[segment_idx] : 0.0;
    double s_end = arc_lengths[segment_idx + 1];
    double segment_length = s_end - s_start;
    double alpha = (segment_length > 1e-6) ? (current_s - s_start) / segment_length : 0.0;
    alpha = std::max(0.0, std::min(1.0, alpha));  // Clamp

    // Interpolate pose
    TrajectoryPoint tp;
    tp.pose = utils::lerp(smoothed_path[segment_idx], smoothed_path[segment_idx + 1], alpha);

    // Compute orientation from direction of travel
    if (segment_idx + 1 < smoothed_path.size()) {
      tp.pose.theta = utils::computeOrientation(
        smoothed_path[segment_idx],
        smoothed_path[segment_idx + 1]);
    }

    // Compute curvature (interpolated)
    double kappa = 0.0;
    if (segment_idx + 1 < curvatures.size()) {
      kappa = curvatures[segment_idx] * (1.0 - alpha) + curvatures[segment_idx + 1] * alpha;
    }
    tp.curvature = kappa;

    // Compute velocity based on curvature and acceleration limits
    double v_target = v_max_;
    if (curvature_velocity_scaling_ && std::abs(kappa) > 1e-6) {
      // Limit velocity based on curvature: v <= sqrt(a_max / |kappa|)
      double v_curve = std::sqrt(a_max_ / std::abs(kappa));
      v_target = std::min(v_max_, v_curve);
    }

    // Trapezoidal velocity profile
    if (current_v < v_target) {
      // Acceleration phase
      double dt = (v_target - current_v) / a_max_;
      current_v = std::min(v_target, current_v + a_max_ * dt);
    } else if (current_v > v_target) {
      // Deceleration phase
      double dt = (current_v - v_target) / a_max_;
      current_v = std::max(v_target, current_v - a_max_ * dt);
    }

    tp.v = current_v;
    tp.omega = current_v * kappa;
    tp.t = current_t;
    tp.arc_length = current_s;

    trajectory.push_back(tp);

    // Advance along path
    double ds = resolution_;
    current_s += ds;
    current_t += ds / std::max(current_v, 0.01);  // Avoid division by zero
  }

  // Add final point
  if (!trajectory.empty() && trajectory.back().arc_length < total_length - 1e-6) {
    TrajectoryPoint final_tp;
    final_tp.pose = smoothed_path.back();
    final_tp.curvature = computeCurvature(smoothed_path, smoothed_path.size() - 1);
    final_tp.v = 0.0;
    final_tp.omega = 0.0;
    final_tp.t = current_t;
    final_tp.arc_length = total_length;
    trajectory.push_back(final_tp);
  }

  return trajectory;
}

double TrajectoryGenerator::computeCurvature(
  const std::vector<Pose2D> & path,
  size_t idx)
{
  if (path.size() < 3) {
    return 0.0;
  }

  // Three-point method for curvature computation
  size_t i = idx;
  if (i == 0) {
    i = 1;  // Use second point if first
  }
  if (i >= path.size() - 1) {
    i = path.size() - 2;  // Use second-to-last if last
  }

  const Pose2D & p0 = path[i - 1];
  const Pose2D & p1 = path[i];
  const Pose2D & p2 = path[i + 1];

  // Compute vectors
  double dx1 = p1.x - p0.x;
  double dy1 = p1.y - p0.y;
  double dx2 = p2.x - p1.x;
  double dy2 = p2.y - p1.y;

  double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double ds2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

  if (ds1 < 1e-6 || ds2 < 1e-6) {
    return 0.0;
  }

  // Curvature = |dθ/ds|
  double theta1 = std::atan2(dy1, dx1);
  double theta2 = std::atan2(dy2, dx2);
  double dtheta = Pose2D::normalizeAngle(theta2 - theta1);
  double ds = (ds1 + ds2) / 2.0;

  double curvature = std::abs(dtheta / ds);

  return curvature;
}

double TrajectoryGenerator::computeArcLength(const Pose2D & p1, const Pose2D & p2) const
{
  return p1.distanceTo(p2);
}

std::vector<double> TrajectoryGenerator::computeArcLengths(const std::vector<Pose2D> & path) const
{
  std::vector<double> arc_lengths(path.size());
  arc_lengths[0] = 0.0;

  for (size_t i = 1; i < path.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + computeArcLength(path[i - 1], path[i]);
  }

  return arc_lengths;
}

double TrajectoryGenerator::computeVelocityForCurvature(double curvature) const
{
  if (!curvature_velocity_scaling_ || std::abs(curvature) < 1e-6) {
    return v_max_;
  }

  // Limit velocity: v = min(v_max, sqrt(a_max / |kappa|))
  double v_curve = std::sqrt(a_max_ / std::abs(curvature));
  return std::min(v_max_, v_curve);
}

}  // namespace navigation

