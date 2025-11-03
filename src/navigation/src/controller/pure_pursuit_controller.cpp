#include "navigation/controller/pure_pursuit_controller.hpp"
#include <algorithm>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace navigation
{

PurePursuitController::PurePursuitController()
: configured_(false),
  lookahead_time_(1.0),
  min_lookahead_(0.3),
  max_lookahead_(1.5),
  v_max_(0.5),
  omega_max_(1.0),
  curvature_slowdown_radius_(1.0),
  proximity_slowdown_dist_(0.5),
  proximity_gain_(0.8),
  current_v_(0.0)
{
}

void PurePursuitController::configure(
  double lookahead_time,
  double min_lookahead,
  double max_lookahead,
  double v_max,
  double omega_max)
{
  lookahead_time_ = lookahead_time;
  min_lookahead_ = min_lookahead;
  max_lookahead_ = max_lookahead;
  v_max_ = v_max;
  omega_max_ = omega_max;
  configured_ = true;
}

void PurePursuitController::setRegulationParams(
  double curvature_slowdown_radius,
  double proximity_slowdown_dist,
  double proximity_gain)
{
  curvature_slowdown_radius_ = curvature_slowdown_radius;
  proximity_slowdown_dist_ = proximity_slowdown_dist;
  proximity_gain_ = proximity_gain;
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocityCommand(
  const Pose2D & current_pose,
  const std::vector<TrajectoryPoint> & trajectory)
{
  geometry_msgs::msg::Twist cmd;

  if (!configured_ || trajectory.empty()) {
    return cmd;  // Zero velocity
  }

  // Find closest point on trajectory
  size_t closest_idx = findClosestPoint(current_pose, trajectory);

  // Check if we've reached the end
  if (closest_idx >= trajectory.size() - 1) {
    // At or past end of trajectory
    double dist_to_end = current_pose.distanceTo(trajectory.back().pose);
    if (dist_to_end < 0.1) {  // Reached goal
      return cmd;  // Zero velocity
    }
  }

  // Compute adaptive lookahead distance
  double lookahead_distance = computeLookaheadDistance();

  // Find lookahead point
  Pose2D lookahead_point = findLookaheadPoint(trajectory, closest_idx, lookahead_distance);

  // Transform lookahead point to robot frame
  Pose2D lookahead_robot = transformToRobotFrame(lookahead_point, current_pose);

  // Compute curvature from lookahead geometry
  // κ = 2·y_l / L²
  double curvature = 0.0;
  if (lookahead_distance > 1e-6) {
    curvature = 2.0 * lookahead_robot.y / (lookahead_distance * lookahead_distance);
  }

  // Start with maximum velocity
  double v_desired = v_max_;

  // Apply curvature-based regulation
  v_desired = applyCurvatureRegulation(v_desired, curvature);

  // Apply proximity-based regulation (if costmap available)
  if (costmap_) {
    v_desired = applyProximityRegulation(v_desired, current_pose);
  }

  // Collision checking
  if (checkCollision(v_desired, v_desired * curvature, current_pose)) {
    v_desired = 0.0;
    curvature = 0.0;
  }

  // Compute angular velocity
  double omega = v_desired * curvature;
  omega = std::max(-omega_max_, std::min(omega_max_, omega));

  // Set command
  cmd.linear.x = v_desired;
  cmd.angular.z = omega;

  return cmd;
}

size_t PurePursuitController::findClosestPoint(
  const Pose2D & current_pose,
  const std::vector<TrajectoryPoint> & trajectory) const
{
  if (trajectory.empty()) {
    return 0;
  }

  size_t closest_idx = 0;
  double min_dist = current_pose.distanceTo(trajectory[0].pose);

  for (size_t i = 1; i < trajectory.size(); ++i) {
    double dist = current_pose.distanceTo(trajectory[i].pose);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

Pose2D PurePursuitController::findLookaheadPoint(
  const std::vector<TrajectoryPoint> & trajectory,
  size_t closest_idx,
  double lookahead_distance) const
{
  if (closest_idx >= trajectory.size()) {
    return trajectory.back().pose;
  }

  // Search forward from closest point for lookahead distance
  double accumulated_dist = 0.0;
  size_t search_idx = closest_idx;

  while (search_idx + 1 < trajectory.size()) {
    double segment_dist = trajectory[search_idx].pose.distanceTo(
      trajectory[search_idx + 1].pose);
    accumulated_dist += segment_dist;

    if (accumulated_dist >= lookahead_distance) {
      // Interpolate within this segment
      double overshoot = accumulated_dist - lookahead_distance;
      double alpha = 1.0 - (overshoot / segment_dist);
      return utils::lerp(trajectory[search_idx].pose, trajectory[search_idx + 1].pose, alpha);
    }

    search_idx++;
  }

  // Lookahead distance extends beyond trajectory, return last point
  return trajectory.back().pose;
}

Pose2D PurePursuitController::transformToRobotFrame(
  const Pose2D & world_pose,
  const Pose2D & robot_pose) const
{
  Pose2D robot_frame;

  // Translate to robot origin
  double dx = world_pose.x - robot_pose.x;
  double dy = world_pose.y - robot_pose.y;

  // Rotate to robot frame
  double cos_theta = std::cos(-robot_pose.theta);
  double sin_theta = std::sin(-robot_pose.theta);

  robot_frame.x = cos_theta * dx - sin_theta * dy;
  robot_frame.y = sin_theta * dx + cos_theta * dy;
  robot_frame.theta = Pose2D::normalizeAngle(world_pose.theta - robot_pose.theta);

  return robot_frame;
}

double PurePursuitController::computeLookaheadDistance() const
{
  // Adaptive lookahead: L = clamp(v·t_lookahead, L_min, L_max)
  double lookahead = current_v_ * lookahead_time_;
  return std::max(min_lookahead_, std::min(max_lookahead_, lookahead));
}

double PurePursuitController::applyCurvatureRegulation(
  double v_desired,
  double curvature) const
{
  // If curvature exceeds threshold, reduce velocity
  if (std::abs(curvature) > 1.0 / curvature_slowdown_radius_) {
    double v_curve = v_max_ / (curvature_slowdown_radius_ * std::abs(curvature));
    return std::min(v_desired, v_curve);
  }

  return v_desired;
}

double PurePursuitController::applyProximityRegulation(
  double v_desired,
  const Pose2D & current_pose) const
{
  if (!costmap_) {
    return v_desired;
  }

  double dist_to_obstacle = getMinDistanceToObstacle(current_pose);

  if (dist_to_obstacle < proximity_slowdown_dist_) {
    double v_prox = v_max_ * proximity_gain_ * (dist_to_obstacle / proximity_slowdown_dist_);
    return std::min(v_desired, v_prox);
  }

  return v_desired;
}

bool PurePursuitController::checkCollision(
  double v,
  double omega,
  const Pose2D & /*current_pose*/) const
{
  // Simple collision check - in full implementation, this would simulate
  // robot trajectory and check against costmap
  // For now, return false (no collision)
  (void)v;
  (void)omega;
  return false;
}

double PurePursuitController::getMinDistanceToObstacle(const Pose2D & /*pose*/) const
{
  if (!costmap_) {
    return 10.0;  // No obstacles if no costmap
  }

  // Simplified: return distance based on costmap lookup
  // In full implementation, would raycast in multiple directions
  // and find minimum distance to occupied cells
  
  // For now, return a safe distance (implementation would use actual costmap)
  return 2.0;
}

}  // namespace navigation

