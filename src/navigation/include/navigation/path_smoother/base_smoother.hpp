#ifndef NAVIGATION__PATH_SMOOTHER__BASE_SMOOTHER_HPP_
#define NAVIGATION__PATH_SMOOTHER__BASE_SMOOTHER_HPP_

#include <vector>
#include <string>
#include "navigation/types.hpp"

namespace navigation
{

/**
 * @brief Abstract base class for path smoothing algorithms
 */
class SmootherInterface
{
public:
  virtual ~SmootherInterface() = default;

  /**
   * @brief Smooth the given path
   * @param path Input path to smooth (will be modified)
   * @return true if smoothing was successful
   */
  virtual bool smooth(std::vector<Pose2D> & path) = 0;

  /**
   * @brief Configure the smoother with parameters
   * @param params Parameter map (specific to each smoother implementation)
   */
  virtual void configure(const std::vector<std::string> & /*params*/) {}

  /**
   * @brief Check if smoother is configured and ready
   */
  virtual bool isConfigured() const { return true; }
};

}  // namespace navigation

#endif  // NAVIGATION__PATH_SMOOTHER__BASE_SMOOTHER_HPP_

