#include "navigation/path_smoother/simple_smoother.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation
{

SimpleSmoother::SimpleSmoother()
: w_data_(0.2),
  w_smooth_(0.3),
  max_iterations_(1000),
  tolerance_(1e-10),
  configured_(false)
{
}

void SimpleSmoother::configure(double w_data, double w_smooth, int max_iterations, double tolerance)
{
  w_data_ = w_data;
  w_smooth_ = w_smooth;
  max_iterations_ = max_iterations;
  tolerance_ = tolerance;
  configured_ = true;
}

bool SimpleSmoother::smooth(std::vector<Pose2D> & path)
{
  if (path.size() < 3) {
    return false;  // Need at least 3 points to smooth (first and last are anchored)
  }

  if (!configured_) {
    // Use default values if not configured
    configured_ = true;
  }

  // Store original path for data fidelity term
  std::vector<Pose2D> original_path = path;
  std::vector<Pose2D> new_path = path;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    double total_change = 0.0;

    // Skip first and last points (they are anchored)
    for (size_t i = 1; i < path.size() - 1; ++i) {
      double grad_x_data = 0.0;
      double grad_y_data = 0.0;
      double grad_x_smooth = 0.0;
      double grad_y_smooth = 0.0;

      // Compute data fidelity gradient (pull toward original path)
      computeDataGradient(original_path, new_path, i, grad_x_data, grad_y_data);

      // Compute smoothness gradient (pull toward neighbors' average)
      computeSmoothGradient(new_path, i, grad_x_smooth, grad_y_smooth);

      // Combine gradients
      double delta_x = w_data_ * grad_x_data + w_smooth_ * grad_x_smooth;
      double delta_y = w_data_ * grad_y_data + w_smooth_ * grad_y_smooth;

      // Update position
      new_path[i].x += delta_x;
      new_path[i].y += delta_y;

      // Update orientation based on neighbors (smooth orientation changes)
      double theta_prev = new_path[i - 1].theta;
      double theta_next = new_path[i + 1].theta;
      double theta_avg = (theta_prev + theta_next) / 2.0;
      double theta_diff = Pose2D::normalizeAngle(theta_avg - new_path[i].theta);
      new_path[i].theta += w_smooth_ * theta_diff;
      new_path[i].theta = Pose2D::normalizeAngle(new_path[i].theta);

      total_change += std::abs(delta_x) + std::abs(delta_y) + std::abs(theta_diff);
    }

    // Check convergence
    if (total_change < tolerance_) {
      path = new_path;
      return true;
    }

    // Update path for next iteration
    path = new_path;
  }

  // Return true even if max iterations reached (convergence is best effort)
  return true;
}

void SimpleSmoother::computeDataGradient(
  const std::vector<Pose2D> & original_path,
  const std::vector<Pose2D> & smoothed_path,
  size_t idx,
  double & grad_x,
  double & grad_y) const
{
  // Data term: gradient pulls point back toward original position
  grad_x = original_path[idx].x - smoothed_path[idx].x;
  grad_y = original_path[idx].y - smoothed_path[idx].y;
}

void SimpleSmoother::computeSmoothGradient(
  const std::vector<Pose2D> & smoothed_path,
  size_t idx,
  double & grad_x,
  double & grad_y) const
{
  // Smooth term: gradient pulls point toward average of neighbors
  // This is equivalent to minimizing the second derivative
  grad_x = smoothed_path[idx - 1].x + smoothed_path[idx + 1].x - 2.0 * smoothed_path[idx].x;
  grad_y = smoothed_path[idx - 1].y + smoothed_path[idx + 1].y - 2.0 * smoothed_path[idx].y;
}

}  // namespace navigation

