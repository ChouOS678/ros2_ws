#include "marl_nav2_plugins/predicted_risk_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>

#include <pluginlib/class_list_macros.hpp>

namespace marl_nav2_plugins
{

void PredictedRiskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in PredictedRiskLayer::onInitialize");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("obstacle_topic", rclcpp::ParameterValue(std::string("/tracked_obstacles")));
  declareParameter("prediction_horizon_s", rclcpp::ParameterValue(2.0));
  declareParameter("prediction_dt_s", rclcpp::ParameterValue(0.25));
  declareParameter("time_decay", rclcpp::ParameterValue(1.0));
  declareParameter("spatial_sigma", rclcpp::ParameterValue(0.35));
  declareParameter("influence_radius", rclcpp::ParameterValue(0.8));
  declareParameter("max_risk_cost", rclcpp::ParameterValue(180));
  declareParameter("max_track_speed", rclcpp::ParameterValue(3.0));
  declareParameter("velocity_smoothing_alpha", rclcpp::ParameterValue(0.6));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "obstacle_topic", obstacle_topic_);
  node->get_parameter(name_ + "." + "prediction_horizon_s", prediction_horizon_s_);
  node->get_parameter(name_ + "." + "prediction_dt_s", prediction_dt_s_);
  node->get_parameter(name_ + "." + "time_decay", time_decay_);
  node->get_parameter(name_ + "." + "spatial_sigma", spatial_sigma_);
  node->get_parameter(name_ + "." + "influence_radius", influence_radius_);
  node->get_parameter(name_ + "." + "max_risk_cost", max_risk_cost_);
  node->get_parameter(name_ + "." + "max_track_speed", max_track_speed_);
  node->get_parameter(name_ + "." + "velocity_smoothing_alpha", velocity_smoothing_alpha_);

  prediction_horizon_s_ = std::max(0.2, prediction_horizon_s_);
  prediction_dt_s_ = std::max(0.05, prediction_dt_s_);
  spatial_sigma_ = std::max(0.05, spatial_sigma_);
  influence_radius_ = std::max(spatial_sigma_, influence_radius_);
  max_track_speed_ = std::max(0.1, max_track_speed_);
  velocity_smoothing_alpha_ = std::clamp(velocity_smoothing_alpha_, 0.0, 1.0);
  max_risk_cost_ = std::clamp(max_risk_cost_, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE - 1));

  obstacles_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    obstacle_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PredictedRiskLayer::obstaclesCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "PredictedRiskLayer initialized topic=%s horizon=%.2f dt=%.2f",
    obstacle_topic_.c_str(), prediction_horizon_s_, prediction_dt_s_);
}

void PredictedRiskLayer::reset()
{
  std::scoped_lock<std::mutex> lock(data_mutex_);
  tracks_.clear();
  predicted_points_.clear();
  current_ = true;
}

void PredictedRiskLayer::obstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  const rclcpp::Time stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
    ? rclcpp::Clock(RCL_ROS_TIME).now()
    : rclcpp::Time(msg->header.stamp);

  std::scoped_lock<std::mutex> lock(data_mutex_);
  std::unordered_set<int> live_ids;
  live_ids.reserve(msg->poses.size());

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const int id = static_cast<int>(i);
    live_ids.insert(id);
    auto & tr = tracks_[id];
    const double x = msg->poses[i].position.x;
    const double y = msg->poses[i].position.y;

    if (tr.initialized) {
      const double dt = (stamp - tr.stamp).seconds();
      if (dt > 1e-3) {
        const double raw_vx = (x - tr.x) / dt;
        const double raw_vy = (y - tr.y) / dt;
        tr.vx = velocity_smoothing_alpha_ * raw_vx + (1.0 - velocity_smoothing_alpha_) * tr.vx;
        tr.vy = velocity_smoothing_alpha_ * raw_vy + (1.0 - velocity_smoothing_alpha_) * tr.vy;
      }
    }

    tr.x = x;
    tr.y = y;
    tr.stamp = stamp;
    tr.initialized = true;

    const double speed = std::hypot(tr.vx, tr.vy);
    if (speed > max_track_speed_) {
      const double s = max_track_speed_ / speed;
      tr.vx *= s;
      tr.vy *= s;
    }
  }

  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (live_ids.find(it->first) == live_ids.end()) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }

  refreshPredictionsLocked();
}

void PredictedRiskLayer::refreshPredictionsLocked()
{
  predicted_points_.clear();
  for (const auto & kv : tracks_) {
    const auto & tr = kv.second;
    if (!tr.initialized) {
      continue;
    }

    for (double t = 0.0; t <= prediction_horizon_s_ + 1e-6; t += prediction_dt_s_) {
      PredictedPoint pt;
      pt.x = tr.x + tr.vx * t;
      pt.y = tr.y + tr.vy * t;
      pt.t = t;
      predicted_points_.push_back(pt);
    }
  }
}

void PredictedRiskLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::scoped_lock<std::mutex> lock(data_mutex_);
  for (const auto & pt : predicted_points_) {
    *min_x = std::min(*min_x, pt.x - influence_radius_);
    *min_y = std::min(*min_y, pt.y - influence_radius_);
    *max_x = std::max(*max_x, pt.x + influence_radius_);
    *max_y = std::max(*max_y, pt.y + influence_radius_);
  }
}

void PredictedRiskLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::vector<PredictedPoint> points;
  {
    std::scoped_lock<std::mutex> lock(data_mutex_);
    points = predicted_points_;
  }

  for (const auto & pt : points) {
    markPointRisk(master_grid, min_i, min_j, max_i, max_j, pt);
  }
}

void PredictedRiskLayer::markPointRisk(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const PredictedPoint & pt)
{
  unsigned int center_mx = 0;
  unsigned int center_my = 0;
  if (!master_grid.worldToMap(pt.x, pt.y, center_mx, center_my)) {
    return;
  }

  const double res = master_grid.getResolution();
  const int radius_cells = std::max(1, static_cast<int>(std::ceil(influence_radius_ / res)));
  const int cxi = static_cast<int>(center_mx);
  const int cyi = static_cast<int>(center_my);

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    const int my = cyi + dy;
    if (my < min_j || my >= max_j) {
      continue;
    }
    if (my < 0 || my >= static_cast<int>(master_grid.getSizeInCellsY())) {
      continue;
    }
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = cxi + dx;
      if (mx < min_i || mx >= max_i) {
        continue;
      }
      if (mx < 0 || mx >= static_cast<int>(master_grid.getSizeInCellsX())) {
        continue;
      }

      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      const double dist = std::hypot(wx - pt.x, wy - pt.y);
      if (dist > influence_radius_) {
        continue;
      }

      const unsigned char c = riskCost(pt.t, dist);
      const unsigned char old = master_grid.getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
      if (c > old) {
        master_grid.setCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), c);
      }
    }
  }
}

unsigned char PredictedRiskLayer::riskCost(double future_t, double distance) const
{
  const double temporal = std::exp(-time_decay_ * std::max(0.0, future_t));
  const double spatial = std::exp(-0.5 * std::pow(distance / spatial_sigma_, 2.0));
  const double score = temporal * spatial;
  const int cost = static_cast<int>(std::round(max_risk_cost_ * score));
  return static_cast<unsigned char>(std::clamp(cost, 0, max_risk_cost_));
}

}  // namespace marl_nav2_plugins

PLUGINLIB_EXPORT_CLASS(marl_nav2_plugins::PredictedRiskLayer, nav2_costmap_2d::Layer)
