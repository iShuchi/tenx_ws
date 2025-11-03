#ifndef NAVIGATION__PATH_SMOOTHER__SIMPLE_SMOOTHER_HPP_
#define NAVIGATION__PATH_SMOOTHER__SIMPLE_SMOOTHER_HPP_

#include <vector>
#include "navigation/path_smoother/base_smoother.hpp"

namespace navigation
{

/**
 * @brief Simple weighted smoother using gradient descent-like approach
 * Based on Nav2's Simple Smoother algorithm
 */
class SimpleSmoother : public SmootherInterface
{
public:
  SimpleSmoother();
  ~SimpleSmoother() override = default;

  /**
   * @brief Smooth path using weighted gradient descent
   */
  bool smooth(std::vector<Pose2D> & path) override;

  /**
   * @brief Configure smoother parameters
   */
  void configure(double w_data, double w_smooth, int max_iterations, double tolerance);

private:
  double w_data_;           // Weight for data fidelity term (pull toward original)
  double w_smooth_;         // Weight for smoothness term (pull toward neighbors)
  int max_iterations_;      // Maximum iterations for convergence
  double tolerance_;         // Convergence tolerance
  bool configured_;

  /**
   * @brief Compute gradient for data fidelity term
   */
  void computeDataGradient(
    const std::vector<Pose2D> & original_path,
    const std::vector<Pose2D> & smoothed_path,
    size_t idx,
    double & grad_x,
    double & grad_y) const;

  /**
   * @brief Compute gradient for smoothness term
   */
  void computeSmoothGradient(
    const std::vector<Pose2D> & smoothed_path,
    size_t idx,
    double & grad_x,
    double & grad_y) const;
};

}  // namespace navigation

#endif  // NAVIGATION__PATH_SMOOTHER__SIMPLE_SMOOTHER_HPP_

