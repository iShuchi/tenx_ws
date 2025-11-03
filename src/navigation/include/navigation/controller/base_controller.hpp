#ifndef NAVIGATION__CONTROLLER__BASE_CONTROLLER_HPP_
#define NAVIGATION__CONTROLLER__BASE_CONTROLLER_HPP_

#include <vector>
#include <memory>
#include "navigation/types.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace navigation
{

/**
 * @brief Abstract base class for trajectory tracking controllers
 */
class ControllerInterface
{
public:
  virtual ~ControllerInterface() = default;

  /**
   * @brief Compute velocity command to follow trajectory
   * @param current_pose Current robot pose
   * @param trajectory Trajectory to follow
   * @return Velocity command (Twist message)
   */
  virtual geometry_msgs::msg::Twist computeVelocityCommand(
    const Pose2D & current_pose,
    const std::vector<TrajectoryPoint> & trajectory) = 0;

  /**
   * @brief Configure the controller with parameters
   */
  virtual void configure(
    double lookahead_time,
    double min_lookahead,
    double max_lookahead,
    double v_max,
    double omega_max) = 0;

  /**
   * @brief Check if controller is configured and ready
   */
  virtual bool isConfigured() const = 0;

  /**
   * @brief Set current velocity (for adaptive lookahead)
   */
  virtual void setCurrentVelocity(double v) { (void)v; }
};

}  // namespace navigation

#endif  // navigation__CONTROLLER__BASE_CONTROLLER_HPP_

