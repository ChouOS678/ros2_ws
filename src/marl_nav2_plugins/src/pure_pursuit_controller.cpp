#include "marl_nav2_plugins/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace nav2_pure_pursuit_controller
{

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Pure pursuit controller parent node is unavailable");
  }

  node->declare_parameter(name_ + ".desired_linear_vel", desired_linear_vel_);
  node->declare_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node->declare_parameter(name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->declare_parameter(name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->declare_parameter(name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(name_ + ".lookahead_time", lookahead_time_);
  speed_limit_ = desired_linear_vel_;
}

void PurePursuitController::cleanup()
{
  plan_.poses.clear();
  tf_.reset();
  costmap_ros_.reset();
}

void PurePursuitController::activate() {}

void PurePursuitController::deactivate() {}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
}

double PurePursuitController::getLookaheadDistance(
  const geometry_msgs::msg::Twist &) const
{
  return std::clamp(lookahead_dist_, min_lookahead_dist_, max_lookahead_dist_);
}

geometry_msgs::msg::PoseStamped PurePursuitController::getCarrot(
  const geometry_msgs::msg::PoseStamped & pose, double lookahead) const
{
  if (plan_.poses.empty()) {
    return pose;
  }

  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < plan_.poses.size(); ++index) {
    const auto & point = plan_.poses[index].pose.position;
    const double distance = std::hypot(point.x - robot_x, point.y - robot_y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  double travelled = 0.0;
  for (std::size_t index = nearest_index + 1; index < plan_.poses.size(); ++index) {
    const auto & previous = plan_.poses[index - 1].pose.position;
    const auto & current = plan_.poses[index].pose.position;
    travelled += std::hypot(current.x - previous.x, current.y - previous.y);
    if (travelled >= lookahead) {
      return plan_.poses[index];
    }
  }
  return plan_.poses.back();
}

geometry_msgs::msg::TwistStamped PurePursuitController::computePurePursuitCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  double lookahead) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  const auto carrot = getCarrot(pose, lookahead);
  const double heading = tf2::getYaw(pose.pose.orientation);
  const double delta_x = carrot.pose.position.x - pose.pose.position.x;
  const double delta_y = carrot.pose.position.y - pose.pose.position.y;
  const double local_x = std::cos(heading) * delta_x + std::sin(heading) * delta_y;
  const double local_y = -std::sin(heading) * delta_x + std::cos(heading) * delta_y;
  const double distance_squared = std::max(delta_x * delta_x + delta_y * delta_y, 1e-6);
  const double curvature = 2.0 * local_y / distance_squared;

  command.twist.linear.x = speed_limit_;
  command.twist.angular.z = speed_limit_ * curvature;
  if (local_x < 0.0) {
    command.twist.linear.x = 0.0;
  }
  return command;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  return computePurePursuitCommand(pose, velocity, getLookaheadDistance(velocity));
}

void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit < 0.0) {
    speed_limit_ = desired_linear_vel_;
  } else if (percentage) {
    speed_limit_ = desired_linear_vel_ * speed_limit / 100.0;
  } else {
    speed_limit_ = std::min(desired_linear_vel_, speed_limit);
  }
}

double AdaptivePurePursuitController::getLookaheadDistance(
  const geometry_msgs::msg::Twist & velocity) const
{
  const double dynamic_distance = lookahead_dist_ + lookahead_time_ * std::abs(velocity.linear.x);
  return std::clamp(dynamic_distance, min_lookahead_dist_, max_lookahead_dist_);
}

}

PLUGINLIB_EXPORT_CLASS(
  nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(
  nav2_pure_pursuit_controller::AdaptivePurePursuitController, nav2_core::Controller)
