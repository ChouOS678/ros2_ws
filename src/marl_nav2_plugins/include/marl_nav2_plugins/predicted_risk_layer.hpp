#ifndef MARL_NAV2_PLUGINS__PREDICTED_RISK_LAYER_HPP_
#define MARL_NAV2_PLUGINS__PREDICTED_RISK_LAYER_HPP_

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace marl_nav2_plugins
{

class PredictedRiskLayer : public nav2_costmap_2d::Layer
{
public:
  PredictedRiskLayer() = default;
  ~PredictedRiskLayer() override = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}
  void onFootprintChanged() override {}

private:
  struct TrackState
  {
    double x {0.0};
    double y {0.0};
    double vx {0.0};
    double vy {0.0};
    rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
    bool initialized {false};
  };

  struct PredictedPoint
  {
    double x {0.0};
    double y {0.0};
    double t {0.0};  // seconds in future
  };

  void obstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void refreshPredictionsLocked();
  void markPointRisk(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j, const PredictedPoint & pt);
  unsigned char riskCost(double future_t, double distance) const;

  bool enabled_ {true};
  std::string obstacle_topic_ {"/tracked_obstacles"};
  double prediction_horizon_s_ {2.0};
  double prediction_dt_s_ {0.25};
  double time_decay_ {1.0};
  double spatial_sigma_ {0.35};
  double influence_radius_ {0.8};
  double max_track_speed_ {3.0};
  double velocity_smoothing_alpha_ {0.6};
  int max_risk_cost_ {180};

  std::mutex data_mutex_;
  std::unordered_map<int, TrackState> tracks_;
  std::vector<PredictedPoint> predicted_points_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_sub_;
};

}  // namespace marl_nav2_plugins

#endif  // MARL_NAV2_PLUGINS__PREDICTED_RISK_LAYER_HPP_
