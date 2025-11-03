#ifndef NAVIGATION__PATH_SMOOTHER__SAVGOL_SMOOTHER_HPP_
#define NAVIGATION__PATH_SMOOTHER__SAVGOL_SMOOTHER_HPP_

#include <vector>
#include "navigation/path_smoother/base_smoother.hpp"

namespace navigation
{

/**
 * @brief Savitzky-Golay filter smoother for fast path smoothing
 * Based on digital signal processing technique using polynomial fitting
 */
class SavGolSmoother : public SmootherInterface
{
public:
  SavGolSmoother();
  ~SavGolSmoother() override = default;

  /**
   * @brief Smooth path using Savitzky-Golay filter
   */
  bool smooth(std::vector<Pose2D> & path) override;

  /**
   * @brief Configure filter parameters
   */
  void configure(int window_size, int polynomial_order);

private:
  int window_size_;        // Window size (must be odd)
  int polynomial_order_;   // Polynomial order (typically 3)
  bool configured_;

  /**
   * @brief Compute Savitzky-Golay coefficients for given window and order
   */
  std::vector<double> computeCoefficients(int window_size, int poly_order);

  /**
   * @brief Apply filter to a data sequence
   */
  void applyFilter(
    const std::vector<double> & input,
    std::vector<double> & output,
    const std::vector<double> & coefficients) const;

  /**
   * @brief Ensure window size is odd
   */
  int ensureOddWindow(int size) const;
};

}  // namespace navigation

#endif  // NAVIGATION__PATH_SMOOTHER__SAVGOL_SMOOTHER_HPP_

