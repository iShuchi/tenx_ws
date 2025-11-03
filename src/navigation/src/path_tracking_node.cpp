#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"

#include "navigation/types.hpp"
#include "navigation/path_smoother/simple_smoother.hpp"
#include "navigation/path_smoother/savgol_smoother.hpp"
#include "navigation/trajectory_generator/trajectory_generator.hpp"
#include "navigation/controller/pure_pursuit_controller.hpp"

namespace navigation
{

class PathTrackingNode : public rclcpp::Node
{
public:
  PathTrackingNode()
  : Node("path_tracking_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare parameters with default values (from config file)
    declare_parameters();
    load_parameters();

    // Initialize components
    initialize_smoother();
    trajectory_generator_ = std::make_shared<TrajectoryGenerator>();
    controller_ = std::make_shared<PurePursuitController>();

    configure_components();

    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "path", 10, std::bind(&PathTrackingNode::pathCallback, this, std::placeholders::_1));

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "costmap", 10, std::bind(&PathTrackingNode::costmapCallback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    original_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("original_path", 10);
    smoothed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("smoothed_path", 10);
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);
    lookahead_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "lookahead_marker", 10);

    // Control timer
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&PathTrackingNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Path tracking node initialized");
  }

private:
  void declare_parameters()
  {
    // Path smoother parameters
    this->declare_parameter<std::string>("path_smoother.type", "simple");
    this->declare_parameter<double>("path_smoother.simple.w_data", 0.2);
    this->declare_parameter<double>("path_smoother.simple.w_smooth", 0.3);
    this->declare_parameter<int>("path_smoother.simple.max_iterations", 1000);
    this->declare_parameter<double>("path_smoother.simple.tolerance", 1e-10);
    this->declare_parameter<int>("path_smoother.savgol.window_size", 7);
    this->declare_parameter<int>("path_smoother.savgol.polynomial_order", 3);

    // Trajectory generator parameters
    this->declare_parameter<double>("trajectory_generator.resolution", 0.05);
    this->declare_parameter<double>("trajectory_generator.v_max", 0.5);
    this->declare_parameter<double>("trajectory_generator.a_max", 0.5);
    this->declare_parameter<bool>("trajectory_generator.curvature_velocity_scaling", true);

    // Controller parameters
    this->declare_parameter<double>("controller.lookahead_time", 1.0);
    this->declare_parameter<double>("controller.min_lookahead", 0.3);
    this->declare_parameter<double>("controller.max_lookahead", 1.5);
    this->declare_parameter<double>("controller.curvature_slowdown_radius", 1.0);
    this->declare_parameter<double>("controller.proximity_slowdown_dist", 0.5);
    this->declare_parameter<double>("controller.proximity_gain", 0.8);
    this->declare_parameter<double>("controller.v_max", 0.5);
    this->declare_parameter<double>("controller.omega_max", 1.0);
    this->declare_parameter<bool>("controller.use_collision_checking", true);

    // Visualization parameters
    this->declare_parameter<bool>("visualization.publish_paths", true);
    this->declare_parameter<bool>("visualization.publish_trajectory", true);
    this->declare_parameter<bool>("visualization.publish_lookahead", true);

    // Frame names
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("robot_frame", "base_link");
  }

  void load_parameters()
  {
    smoother_type_ = this->get_parameter("path_smoother.type").as_string();
    smoother_w_data_ = this->get_parameter("path_smoother.simple.w_data").as_double();
    smoother_w_smooth_ = this->get_parameter("path_smoother.simple.w_smooth").as_double();
    smoother_max_iterations_ = this->get_parameter("path_smoother.simple.max_iterations").as_int();
    smoother_tolerance_ = this->get_parameter("path_smoother.simple.tolerance").as_double();
    savgol_window_size_ = this->get_parameter("path_smoother.savgol.window_size").as_int();
    savgol_polynomial_order_ = this->get_parameter("path_smoother.savgol.polynomial_order").as_int();

    traj_resolution_ = this->get_parameter("trajectory_generator.resolution").as_double();
    traj_v_max_ = this->get_parameter("trajectory_generator.v_max").as_double();
    traj_a_max_ = this->get_parameter("trajectory_generator.a_max").as_double();
    traj_curvature_scaling_ = this->get_parameter(
      "trajectory_generator.curvature_velocity_scaling").as_bool();

    ctrl_lookahead_time_ = this->get_parameter("controller.lookahead_time").as_double();
    ctrl_min_lookahead_ = this->get_parameter("controller.min_lookahead").as_double();
    ctrl_max_lookahead_ = this->get_parameter("controller.max_lookahead").as_double();
    ctrl_curvature_radius_ = this->get_parameter("controller.curvature_slowdown_radius").as_double();
    ctrl_proximity_dist_ = this->get_parameter("controller.proximity_slowdown_dist").as_double();
    ctrl_proximity_gain_ = this->get_parameter("controller.proximity_gain").as_double();
    ctrl_v_max_ = this->get_parameter("controller.v_max").as_double();
    ctrl_omega_max_ = this->get_parameter("controller.omega_max").as_double();

    publish_paths_ = this->get_parameter("visualization.publish_paths").as_bool();
    publish_trajectory_ = this->get_parameter("visualization.publish_trajectory").as_bool();
    publish_lookahead_ = this->get_parameter("visualization.publish_lookahead").as_bool();

    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
  }

  void initialize_smoother()
  {
    if (smoother_type_ == "simple") {
      smoother_ = std::make_shared<SimpleSmoother>();
      auto simple_smoother = std::static_pointer_cast<SimpleSmoother>(smoother_);
      simple_smoother->configure(
        smoother_w_data_, smoother_w_smooth_, smoother_max_iterations_, smoother_tolerance_);
    } else if (smoother_type_ == "savgol") {
      smoother_ = std::make_shared<SavGolSmoother>();
      auto savgol_smoother = std::static_pointer_cast<SavGolSmoother>(smoother_);
      savgol_smoother->configure(savgol_window_size_, savgol_polynomial_order_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown smoother type: %s", smoother_type_.c_str());
      smoother_ = std::make_shared<SimpleSmoother>();  // Default
    }
  }

  void configure_components()
  {
    trajectory_generator_->configure(
      traj_resolution_, traj_v_max_, traj_a_max_, traj_curvature_scaling_);

    controller_->configure(
      ctrl_lookahead_time_,
      ctrl_min_lookahead_,
      ctrl_max_lookahead_,
      ctrl_v_max_,
      ctrl_omega_max_);

    controller_->setRegulationParams(
      ctrl_curvature_radius_,
      ctrl_proximity_dist_,
      ctrl_proximity_gain_);
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;
    }

    // Convert nav_msgs::Path to vector<Pose2D>
    original_path_.clear();
    for (const auto & pose_stamped : msg->poses) {
      Pose2D pose;
      pose.x = pose_stamped.pose.position.x;
      pose.y = pose_stamped.pose.position.y;
      pose.theta = 2.0 * std::atan2(
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w);  // Quaternion to yaw
      original_path_.push_back(pose);
    }

    // Smooth path
    smoothed_path_ = original_path_;
    if (smoother_ && smoother_->smooth(smoothed_path_)) {
      // Generate trajectory
      trajectory_ = trajectory_generator_->generateTrajectory(smoothed_path_);
      trajectory_ready_ = true;

      // Publish visualization
      if (publish_paths_) {
        publishPaths();
      }
      if (publish_trajectory_) {
        publishTrajectory();
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Path smoothing failed");
    }
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    costmap_ = msg;
    if (controller_) {
      controller_->setCostmap(costmap_);
    }
  }

  void controlLoop()
  {
    if (!trajectory_ready_ || trajectory_.empty()) {
      return;
    }

    // Get current robot pose from TF
    Pose2D current_pose;
    if (!getCurrentPose(current_pose)) {
      return;
    }

    // Update controller with current velocity
    if (last_cmd_) {
      controller_->setCurrentVelocity(last_cmd_->linear.x);
    }

    // Compute velocity command
    auto cmd = controller_->computeVelocityCommand(current_pose, trajectory_);
    last_cmd_ = std::make_shared<geometry_msgs::msg::Twist>(cmd);

    // Publish command
    cmd_vel_pub_->publish(cmd);

    // Publish lookahead marker
    if (publish_lookahead_) {
      publishLookaheadMarker(current_pose);
    }
  }

  bool getCurrentPose(Pose2D & pose)
  {
    try {
      auto transform = tf_buffer_.lookupTransform(
        global_frame_, robot_frame_, rclcpp::Time(0));

      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.theta = 2.0 * std::atan2(
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      return true;
    } catch (const tf2::TransformException &) {
      // TF lookup failed - robot pose not available yet
      return false;
    }
  }

  void publishPaths()
  {
    if (original_path_pub_->get_subscription_count() > 0) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = global_frame_;
      path_msg.header.stamp = this->now();

      for (const auto & pose : original_path_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = pose.x;
        ps.pose.position.y = pose.y;
        ps.pose.orientation.z = std::sin(pose.theta / 2.0);
        ps.pose.orientation.w = std::cos(pose.theta / 2.0);
        path_msg.poses.push_back(ps);
      }
      original_path_pub_->publish(path_msg);
    }

    if (smoothed_path_pub_->get_subscription_count() > 0) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = global_frame_;
      path_msg.header.stamp = this->now();

      for (const auto & pose : smoothed_path_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = pose.x;
        ps.pose.position.y = pose.y;
        ps.pose.orientation.z = std::sin(pose.theta / 2.0);
        ps.pose.orientation.w = std::cos(pose.theta / 2.0);
        path_msg.poses.push_back(ps);
      }
      smoothed_path_pub_->publish(path_msg);
    }
  }

  void publishTrajectory()
  {
    if (trajectory_pub_->get_subscription_count() == 0) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;

    // Color-code by velocity (blue = slow, red = fast)
    double v_min = 0.0;
    double v_max = traj_v_max_;

    for (const auto & tp : trajectory_) {
      geometry_msgs::msg::Point pt;
      pt.x = tp.pose.x;
      pt.y = tp.pose.y;
      pt.z = 0.0;
      marker.points.push_back(pt);

      // Color based on velocity
      std_msgs::msg::ColorRGBA color;
      double normalized_v = (tp.v - v_min) / (v_max - v_min + 1e-6);
      color.r = normalized_v;
      color.g = 0.0;
      color.b = 1.0 - normalized_v;
      color.a = 1.0;
      marker.colors.push_back(color);
    }

    marker_array.markers.push_back(marker);
    trajectory_pub_->publish(marker_array);
  }

  void publishLookaheadMarker(const Pose2D & current_pose)
  {
    if (trajectory_.empty() || lookahead_pub_->get_subscription_count() == 0) {
      return;
    }

    // Find lookahead point (simplified - would use controller's logic)
    size_t closest_idx = 0;
    double min_dist = current_pose.distanceTo(trajectory_[0].pose);
    for (size_t i = 1; i < trajectory_.size(); ++i) {
      double dist = current_pose.distanceTo(trajectory_[i].pose);
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }

    size_t lookahead_idx = std::min(
      closest_idx + static_cast<size_t>(5), trajectory_.size() - 1);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->now();
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = trajectory_[lookahead_idx].pose.x;
    marker.pose.position.y = trajectory_[lookahead_idx].pose.y;
    marker.pose.position.z = 0.1;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    lookahead_pub_->publish(marker);
  }

  // Parameters
  std::string smoother_type_;
  double smoother_w_data_, smoother_w_smooth_, smoother_tolerance_;
  int smoother_max_iterations_;
  int savgol_window_size_, savgol_polynomial_order_;
  double traj_resolution_, traj_v_max_, traj_a_max_;
  bool traj_curvature_scaling_;
  double ctrl_lookahead_time_, ctrl_min_lookahead_, ctrl_max_lookahead_;
  double ctrl_curvature_radius_, ctrl_proximity_dist_, ctrl_proximity_gain_;
  double ctrl_v_max_, ctrl_omega_max_;
  bool publish_paths_, publish_trajectory_, publish_lookahead_;
  std::string global_frame_, robot_frame_;

  // Components
  std::shared_ptr<SmootherInterface> smoother_;
  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  std::shared_ptr<PurePursuitController> controller_;

  // State
  std::vector<Pose2D> original_path_;
  std::vector<Pose2D> smoothed_path_;
  std::vector<TrajectoryPoint> trajectory_;
  bool trajectory_ready_{false};
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  geometry_msgs::msg::Twist::SharedPtr last_cmd_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::PathTrackingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

