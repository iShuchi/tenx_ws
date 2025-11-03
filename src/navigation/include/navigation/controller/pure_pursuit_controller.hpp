#ifndef NAVIGATION__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAVIGATION__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <vector>
#include <memory>
#include "navigation/controller/base_controller.hpp"
#include "navigation/types.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace navigation
{

/**
 * @brief Regulated Pure Pursuit Controller (Nav2-style)
 * Implements adaptive lookahead distance with curvature and proximity regulation
 */
class PurePursuitController : public ControllerInterface
{
public:
  PurePursuitController();
  ~PurePursuitController() override = default;

  /**
   * @brief Compute velocity command using Pure Pursuit
   */
  geometry_msgs::msg::Twist computeVelocityCommand(
    const Pose2D & current_pose,
    const std::vector<TrajectoryPoint> & trajectory) override;

  /**
   * @brief Configure controller parameters
   */
  void configure(
    double lookahead_time,
    double min_lookahead,
    double max_lookahead,
    double v_max,
    double omega_max) override;

  /**
   * @brief Check if configured
   */
  bool isConfigured() const override { return configured_; }

  /**
   * @brief Set current velocity for adaptive lookahead
   */
  void setCurrentVelocity(double v) override { current_v_ = v; }

  /**
   * @brief Configure velocity regulation parameters
   */
  void setRegulationParams(
    double curvature_slowdown_radius,
    double proximity_slowdown_dist,
    double proximity_gain);

  /**
   * @brief Set costmap for collision checking (optional)
   */
  void setCostmap(std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap)
  {
    costmap_ = costmap;
  }

private:
  bool configured_;
  double lookahead_time_;
  double min_lookahead_;
  double max_lookahead_;
  double v_max_;
  double omega_max_;
  double curvature_slowdown_radius_;
  double proximity_slowdown_dist_;
  double proximity_gain_;
  double current_v_;

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_;

  /**
   * @brief Find closest point on trajectory to current pose
   */
  size_t findClosestPoint(
    const Pose2D & current_pose,
    const std::vector<TrajectoryPoint> & trajectory) const;

  /**
   * @brief Find lookahead point along trajectory
   */
  Pose2D findLookaheadPoint(
    const std::vector<TrajectoryPoint> & trajectory,
    size_t closest_idx,
    double lookahead_distance) const;

  /**
   * @brief Transform pose to robot frame
   */
  Pose2D transformToRobotFrame(const Pose2D & world_pose, const Pose2D & robot_pose) const;

  /**
   * @brief Compute adaptive lookahead distance
   */
  double computeLookaheadDistance() const;

  /**
   * @brief Apply curvature-based velocity regulation
   */
  double applyCurvatureRegulation(double v_desired, double curvature) const;

  /**
   * @brief Apply proximity-based velocity regulation
   */
  double applyProximityRegulation(double v_desired, const Pose2D & current_pose) const;

  /**
   * @brief Check for collisions along path
   */
  bool checkCollision(double v, double omega, const Pose2D & current_pose) const;

  /**
   * @brief Get minimum distance to obstacle from costmap
   */
  double getMinDistanceToObstacle(const Pose2D & pose) const;
};

}  // namespace navigation

#endif  // NAVIGATION__CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

