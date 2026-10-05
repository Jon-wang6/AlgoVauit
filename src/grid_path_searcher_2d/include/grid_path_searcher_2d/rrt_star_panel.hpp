/**
 * @file rrt_star_panel.hpp
 * @brief RViz2 中实时调整全部二维规划算法参数的面板声明。
 *
 * 文件职责：按地图、A*、TimeBreak A*、JPS、PRM、RRT、RRT* 及三种 RRT*
 * 变体分组定义控件，并通过 ROS 2 参数客户端把数值发送给 /demo_node_2d。
 *
 * 包含的类：RrtStarPanel，继承 rviz_common::Panel。
 * 包含的函数：RrtStarPanel()、onInitialize()、scheduleUpdate()、
 * applyParameters()。
 */
#pragma once

#include <memory>

#include "rclcpp/parameter_client.hpp"
#include "rviz_common/panel.hpp"

class QDoubleSpinBox;
class QCheckBox;
class QLabel;
class QSpinBox;
class QTimer;

namespace grid_path_searcher_2d
{

/** 全算法实时参数面板；控件停止变化 250 ms 后自动发送参数。 */
class RrtStarPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RrtStarPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  /** 重启单次定时器，对连续拖动/输入进行防抖。 */
  void scheduleUpdate();

  /** 收集全部控件数值并发送到 demo_node_2d 参数服务。 */
  void applyParameters();

private:
  QSpinBox * occupied_threshold_{nullptr};
  QCheckBox * allow_unknown_{nullptr};
  QDoubleSpinBox * astar_weight_{nullptr};
  QDoubleSpinBox * timed_astar_weight_{nullptr};
  QDoubleSpinBox * timed_astar_limit_{nullptr};
  QDoubleSpinBox * jps_weight_{nullptr};
  QSpinBox * prm_samples_{nullptr};
  QSpinBox * prm_neighbors_{nullptr};
  QSpinBox * prm_seed_{nullptr};
  QSpinBox * rrt_iterations_{nullptr};
  QDoubleSpinBox * rrt_step_size_{nullptr};
  QDoubleSpinBox * rrt_goal_bias_{nullptr};
  QSpinBox * rrt_seed_{nullptr};
  QSpinBox * rrt_star_iterations_{nullptr};
  QDoubleSpinBox * rrt_star_step_size_{nullptr};
  QDoubleSpinBox * rrt_star_rewire_radius_{nullptr};
  QDoubleSpinBox * rrt_star_goal_bias_{nullptr};
  QSpinBox * rrt_star_seed_{nullptr};
  QSpinBox * kinodynamic_iterations_{nullptr};
  QDoubleSpinBox * kinodynamic_time_step_{nullptr};
  QDoubleSpinBox * kinodynamic_max_speed_{nullptr};
  QDoubleSpinBox * kinodynamic_max_acceleration_{nullptr};
  QDoubleSpinBox * kinodynamic_rewire_radius_{nullptr};
  QDoubleSpinBox * kinodynamic_goal_bias_{nullptr};
  QSpinBox * kinodynamic_seed_{nullptr};
  QSpinBox * anytime_iterations_{nullptr};
  QDoubleSpinBox * anytime_time_budget_{nullptr};
  QDoubleSpinBox * anytime_step_size_{nullptr};
  QDoubleSpinBox * anytime_rewire_radius_{nullptr};
  QDoubleSpinBox * anytime_goal_bias_{nullptr};
  QSpinBox * anytime_seed_{nullptr};
  QSpinBox * informed_iterations_{nullptr};
  QDoubleSpinBox * informed_step_size_{nullptr};
  QDoubleSpinBox * informed_rewire_radius_{nullptr};
  QDoubleSpinBox * informed_goal_bias_{nullptr};
  QSpinBox * informed_seed_{nullptr};
  QLabel * status_{nullptr};
  QTimer * debounce_timer_{nullptr};
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
};

}  // namespace grid_path_searcher_2d
