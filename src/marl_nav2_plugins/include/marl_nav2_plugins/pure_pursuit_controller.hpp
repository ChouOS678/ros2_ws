#ifndef MARL_NAV2_PLUGINS__PURE_PURSUIT_CONTROLLER_HPP_
#define MARL_NAV2_PLUGINS__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_pure_pursuit_controller
{

class PurePursuitController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  virtual double getLookaheadDistance(const geometry_msgs::msg::Twist & velocity) const;
  geometry_msgs::msg::PoseStamped getCarrot(
    const geometry_msgs::msg::PoseStamped & pose, double lookahead) const;
  geometry_msgs::msg::TwistStamped computePurePursuitCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    double lookahead) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path plan_;
  std::string name_;
  double desired_linear_vel_{0.65};
  double lookahead_dist_{0.8};
  double min_lookahead_dist_{0.35};
  double max_lookahead_dist_{1.2};
  double lookahead_time_{1.5};
  double speed_limit_{0.65};
};

class AdaptivePurePursuitController : public PurePursuitController
{
protected:
  double getLookaheadDistance(const geometry_msgs::msg::Twist & velocity) const override;
};

}

#endif
