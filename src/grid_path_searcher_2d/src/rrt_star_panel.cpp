/**
 * @file rrt_star_panel.cpp
 * @brief 二维全部路径规划算法的 RViz 参数面板实现。
 *
 * 文件职责：用可滚动分组界面展示地图和十种算法的可调参数；控件停止变化
 * 250 ms 后统一发送到 /demo_node_2d 并触发重规划。
 * Dijkstra 没有启发式或随机参数，因此只显示说明文字。
 */
#include "grid_path_searcher_2d/rrt_star_panel.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QScrollArea>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace grid_path_searcher_2d
{
namespace
{

/** 创建整数控件并统一设置范围、步长和初值。 */
QSpinBox * makeIntegerControl(
  QWidget * parent, const int minimum, const int maximum,
  const int step, const int value)
{
  auto * control = new QSpinBox(parent);
  control->setRange(minimum, maximum);
  control->setSingleStep(step);
  control->setValue(value);
  return control;
}

/** 创建浮点控件并统一设置范围、精度、步长、初值和单位。 */
QDoubleSpinBox * makeDoubleControl(
  QWidget * parent, const double minimum, const double maximum,
  const double step, const double value, const int decimals,
  const QString & suffix = QString())
{
  auto * control = new QDoubleSpinBox(parent);
  control->setRange(minimum, maximum);
  control->setSingleStep(step);
  control->setDecimals(decimals);
  control->setValue(value);
  control->setSuffix(suffix);
  return control;
}

}  // namespace

RrtStarPanel::RrtStarPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * content = new QWidget(this);
  auto * content_layout = new QVBoxLayout(content);

  auto * map_group = new QGroupBox("通用地图规则", content);
  auto * map_form = new QFormLayout(map_group);
  occupied_threshold_ = makeIntegerControl(map_group, 1, 100, 1, 50);
  allow_unknown_ = new QCheckBox("允许经过未知格", map_group);
  map_form->addRow("占据阈值", occupied_threshold_);
  map_form->addRow("未知格", allow_unknown_);
  content_layout->addWidget(map_group);

  auto * astar_group = new QGroupBox("A* / Dijkstra", content);
  auto * astar_form = new QFormLayout(astar_group);
  astar_weight_ = makeDoubleControl(astar_group, 0.0, 5.0, 0.1, 1.0, 2);
  auto * dijkstra_hint = new QLabel("Dijkstra 只按 g 排序，没有独立参数。", astar_group);
  dijkstra_hint->setWordWrap(true);
  astar_form->addRow("A* 启发权重", astar_weight_);
  astar_form->addRow(dijkstra_hint);
  content_layout->addWidget(astar_group);

  auto * timed_group = new QGroupBox("TimeBreak A*", content);
  auto * timed_form = new QFormLayout(timed_group);
  timed_astar_weight_ = makeDoubleControl(timed_group, 0.0, 5.0, 0.1, 1.0, 2);
  timed_astar_limit_ = makeDoubleControl(timed_group, 0.05, 1000.0, 0.25, 2.0, 2, " ms");
  timed_form->addRow("启发权重", timed_astar_weight_);
  timed_form->addRow("时间上限", timed_astar_limit_);
  content_layout->addWidget(timed_group);

  auto * jps_group = new QGroupBox("JPS", content);
  auto * jps_form = new QFormLayout(jps_group);
  jps_weight_ = makeDoubleControl(jps_group, 0.0, 5.0, 0.1, 1.0, 2);
  jps_form->addRow("启发权重", jps_weight_);
  content_layout->addWidget(jps_group);

  auto * prm_group = new QGroupBox("PRM", content);
  auto * prm_form = new QFormLayout(prm_group);
  prm_samples_ = makeIntegerControl(prm_group, 10, 10000, 50, 500);
  prm_neighbors_ = makeIntegerControl(prm_group, 1, 200, 1, 15);
  prm_seed_ = makeIntegerControl(prm_group, 0, 1000000000, 1, 23);
  prm_form->addRow("采样节点数", prm_samples_);
  prm_form->addRow("近邻连接数", prm_neighbors_);
  prm_form->addRow("随机种子", prm_seed_);
  content_layout->addWidget(prm_group);

  auto * rrt_group = new QGroupBox("RRT", content);
  auto * rrt_form = new QFormLayout(rrt_group);
  rrt_iterations_ = makeIntegerControl(rrt_group, 100, 100000, 500, 8000);
  rrt_step_size_ = makeDoubleControl(rrt_group, 0.5, 30.0, 0.5, 5.0, 1, " 格");
  rrt_goal_bias_ = makeDoubleControl(rrt_group, 0.0, 1.0, 0.01, 0.12, 2);
  rrt_seed_ = makeIntegerControl(rrt_group, 0, 1000000000, 1, 31);
  rrt_form->addRow("最大迭代次数", rrt_iterations_);
  rrt_form->addRow("扩展步长", rrt_step_size_);
  rrt_form->addRow("目标偏置", rrt_goal_bias_);
  rrt_form->addRow("随机种子", rrt_seed_);
  content_layout->addWidget(rrt_group);

  auto * rrt_star_group = new QGroupBox("RRT*", content);
  auto * rrt_star_form = new QFormLayout(rrt_star_group);
  rrt_star_iterations_ = makeIntegerControl(rrt_star_group, 100, 100000, 500, 5000);
  rrt_star_step_size_ =
    makeDoubleControl(rrt_star_group, 0.5, 30.0, 0.5, 5.0, 1, " 格");
  rrt_star_rewire_radius_ =
    makeDoubleControl(rrt_star_group, 1.0, 50.0, 1.0, 12.0, 1, " 格");
  rrt_star_goal_bias_ = makeDoubleControl(rrt_star_group, 0.0, 1.0, 0.01, 0.12, 2);
  rrt_star_seed_ = makeIntegerControl(rrt_star_group, 0, 1000000000, 1, 37);
  rrt_star_form->addRow("最大迭代次数", rrt_star_iterations_);
  rrt_star_form->addRow("扩展步长", rrt_star_step_size_);
  rrt_star_form->addRow("重连半径", rrt_star_rewire_radius_);
  rrt_star_form->addRow("目标偏置", rrt_star_goal_bias_);
  rrt_star_form->addRow("随机种子", rrt_star_seed_);
  content_layout->addWidget(rrt_star_group);

  auto * kinodynamic_group = new QGroupBox("Kinodynamic-RRT*", content);
  auto * kinodynamic_form = new QFormLayout(kinodynamic_group);
  kinodynamic_iterations_ = makeIntegerControl(kinodynamic_group, 100, 100000, 500, 6000);
  kinodynamic_time_step_ =
    makeDoubleControl(kinodynamic_group, 0.05, 5.0, 0.05, 0.8, 2, " s");
  kinodynamic_max_speed_ =
    makeDoubleControl(kinodynamic_group, 0.1, 30.0, 0.5, 6.0, 1, " 格/s");
  kinodynamic_max_acceleration_ =
    makeDoubleControl(kinodynamic_group, 0.1, 30.0, 0.5, 4.0, 1, " 格/s²");
  kinodynamic_rewire_radius_ =
    makeDoubleControl(kinodynamic_group, 1.0, 50.0, 1.0, 10.0, 1, " 格");
  kinodynamic_goal_bias_ =
    makeDoubleControl(kinodynamic_group, 0.0, 1.0, 0.01, 0.15, 2);
  kinodynamic_seed_ = makeIntegerControl(kinodynamic_group, 0, 1000000000, 1, 41);
  kinodynamic_form->addRow("最大迭代次数", kinodynamic_iterations_);
  kinodynamic_form->addRow("推进时间步", kinodynamic_time_step_);
  kinodynamic_form->addRow("最大速度", kinodynamic_max_speed_);
  kinodynamic_form->addRow("最大加速度", kinodynamic_max_acceleration_);
  kinodynamic_form->addRow("重连半径", kinodynamic_rewire_radius_);
  kinodynamic_form->addRow("目标偏置", kinodynamic_goal_bias_);
  kinodynamic_form->addRow("随机种子", kinodynamic_seed_);
  content_layout->addWidget(kinodynamic_group);

  auto * anytime_group = new QGroupBox("Anytime-RRT*", content);
  auto * anytime_form = new QFormLayout(anytime_group);
  anytime_iterations_ = makeIntegerControl(anytime_group, 100, 200000, 1000, 12000);
  anytime_time_budget_ =
    makeDoubleControl(anytime_group, 1.0, 5000.0, 10.0, 200.0, 1, " ms");
  anytime_step_size_ = makeDoubleControl(anytime_group, 0.5, 30.0, 0.5, 5.0, 1, " 格");
  anytime_rewire_radius_ =
    makeDoubleControl(anytime_group, 1.0, 50.0, 1.0, 12.0, 1, " 格");
  anytime_goal_bias_ = makeDoubleControl(anytime_group, 0.0, 1.0, 0.01, 0.12, 2);
  anytime_seed_ = makeIntegerControl(anytime_group, 0, 1000000000, 1, 43);
  anytime_form->addRow("最大迭代次数", anytime_iterations_);
  anytime_form->addRow("时间预算", anytime_time_budget_);
  anytime_form->addRow("扩展步长", anytime_step_size_);
  anytime_form->addRow("重连半径", anytime_rewire_radius_);
  anytime_form->addRow("目标偏置", anytime_goal_bias_);
  anytime_form->addRow("随机种子", anytime_seed_);
  content_layout->addWidget(anytime_group);

  auto * informed_group = new QGroupBox("Informed RRT*", content);
  auto * informed_form = new QFormLayout(informed_group);
  informed_iterations_ = makeIntegerControl(informed_group, 100, 100000, 500, 7000);
  informed_step_size_ = makeDoubleControl(informed_group, 0.5, 30.0, 0.5, 5.0, 1, " 格");
  informed_rewire_radius_ =
    makeDoubleControl(informed_group, 1.0, 50.0, 1.0, 12.0, 1, " 格");
  informed_goal_bias_ = makeDoubleControl(informed_group, 0.0, 1.0, 0.01, 0.12, 2);
  informed_seed_ = makeIntegerControl(informed_group, 0, 1000000000, 1, 47);
  informed_form->addRow("最大迭代次数", informed_iterations_);
  informed_form->addRow("扩展步长", informed_step_size_);
  informed_form->addRow("重连半径", informed_rewire_radius_);
  informed_form->addRow("目标偏置", informed_goal_bias_);
  informed_form->addRow("随机种子", informed_seed_);
  content_layout->addWidget(informed_group);

  auto * weight_hint = new QLabel(
    "启发权重：0 等同 Dijkstra 式排序，1 是标准算法，>1 更激进但不保证最短。",
    content);
  weight_hint->setWordWrap(true);
  content_layout->addWidget(weight_hint);
  status_ = new QLabel("等待 RViz 初始化…", content);
  status_->setWordWrap(true);
  content_layout->addWidget(status_);
  content_layout->addStretch();

  auto * scroll_area = new QScrollArea(this);
  scroll_area->setWidgetResizable(true);
  scroll_area->setWidget(content);
  auto * root_layout = new QVBoxLayout();
  root_layout->setContentsMargins(0, 0, 0, 0);
  root_layout->addWidget(scroll_area);
  setLayout(root_layout);
  setMinimumWidth(300);

  debounce_timer_ = new QTimer(this);
  debounce_timer_->setSingleShot(true);
  debounce_timer_->setInterval(250);
  connect(debounce_timer_, &QTimer::timeout, this, &RrtStarPanel::applyParameters);

  const auto connect_integer = [this](QSpinBox * control) {
      connect(control, qOverload<int>(&QSpinBox::valueChanged), this,
        [this](int) {scheduleUpdate();});
    };
  const auto connect_double = [this](QDoubleSpinBox * control) {
      connect(control, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [this](double) {scheduleUpdate();});
    };
  connect_integer(occupied_threshold_);
  connect(allow_unknown_, &QCheckBox::toggled, this, [this](bool) {scheduleUpdate();});
  connect_double(astar_weight_);
  connect_double(timed_astar_weight_);
  connect_double(timed_astar_limit_);
  connect_double(jps_weight_);
  connect_integer(prm_samples_);
  connect_integer(prm_neighbors_);
  connect_integer(prm_seed_);
  connect_integer(rrt_iterations_);
  connect_double(rrt_step_size_);
  connect_double(rrt_goal_bias_);
  connect_integer(rrt_seed_);
  connect_integer(rrt_star_iterations_);
  connect_double(rrt_star_step_size_);
  connect_double(rrt_star_rewire_radius_);
  connect_double(rrt_star_goal_bias_);
  connect_integer(rrt_star_seed_);
  connect_integer(kinodynamic_iterations_);
  connect_double(kinodynamic_time_step_);
  connect_double(kinodynamic_max_speed_);
  connect_double(kinodynamic_max_acceleration_);
  connect_double(kinodynamic_rewire_radius_);
  connect_double(kinodynamic_goal_bias_);
  connect_integer(kinodynamic_seed_);
  connect_integer(anytime_iterations_);
  connect_double(anytime_time_budget_);
  connect_double(anytime_step_size_);
  connect_double(anytime_rewire_radius_);
  connect_double(anytime_goal_bias_);
  connect_integer(anytime_seed_);
  connect_integer(informed_iterations_);
  connect_double(informed_step_size_);
  connect_double(informed_rewire_radius_);
  connect_double(informed_goal_bias_);
  connect_integer(informed_seed_);
}

void RrtStarPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_->setText("无法取得 RViz ROS 节点");
    return;
  }
  node_ = abstraction->get_raw_node();
  parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/demo_node_2d");
  status_->setText("已连接 /demo_node_2d；停止输入 250 ms 后自动应用并重新规划");
}

void RrtStarPanel::scheduleUpdate()
{
  if (debounce_timer_) {debounce_timer_->start();}
}

void RrtStarPanel::applyParameters()
{
  if (!parameter_client_) {
    status_->setText("参数客户端尚未初始化");
    return;
  }
  if (!parameter_client_->service_is_ready()) {
    status_->setText("/demo_node_2d 参数服务尚未就绪，请稍后再调整");
    return;
  }

  const std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("map.occupied_threshold", occupied_threshold_->value()),
    rclcpp::Parameter("map.allow_unknown", allow_unknown_->isChecked()),
    rclcpp::Parameter("astar.heuristic_weight", astar_weight_->value()),
    rclcpp::Parameter("timed_astar.heuristic_weight", timed_astar_weight_->value()),
    rclcpp::Parameter("timed_astar.time_limit_ms", timed_astar_limit_->value()),
    rclcpp::Parameter("jps.heuristic_weight", jps_weight_->value()),
    rclcpp::Parameter("prm.sample_count", prm_samples_->value()),
    rclcpp::Parameter("prm.k_neighbors", prm_neighbors_->value()),
    rclcpp::Parameter("prm.seed", prm_seed_->value()),
    rclcpp::Parameter("rrt.max_iterations", rrt_iterations_->value()),
    rclcpp::Parameter("rrt.step_size", rrt_step_size_->value()),
    rclcpp::Parameter("rrt.goal_bias", rrt_goal_bias_->value()),
    rclcpp::Parameter("rrt.seed", rrt_seed_->value()),
    rclcpp::Parameter("rrt_star.max_iterations", rrt_star_iterations_->value()),
    rclcpp::Parameter("rrt_star.step_size", rrt_star_step_size_->value()),
    rclcpp::Parameter("rrt_star.rewire_radius", rrt_star_rewire_radius_->value()),
    rclcpp::Parameter("rrt_star.goal_bias", rrt_star_goal_bias_->value()),
    rclcpp::Parameter("rrt_star.seed", rrt_star_seed_->value()),
    rclcpp::Parameter("kinodynamic_rrt_star.max_iterations", kinodynamic_iterations_->value()),
    rclcpp::Parameter("kinodynamic_rrt_star.time_step", kinodynamic_time_step_->value()),
    rclcpp::Parameter("kinodynamic_rrt_star.max_speed", kinodynamic_max_speed_->value()),
    rclcpp::Parameter(
      "kinodynamic_rrt_star.max_acceleration", kinodynamic_max_acceleration_->value()),
    rclcpp::Parameter(
      "kinodynamic_rrt_star.rewire_radius", kinodynamic_rewire_radius_->value()),
    rclcpp::Parameter("kinodynamic_rrt_star.goal_bias", kinodynamic_goal_bias_->value()),
    rclcpp::Parameter("kinodynamic_rrt_star.seed", kinodynamic_seed_->value()),
    rclcpp::Parameter("anytime_rrt_star.max_iterations", anytime_iterations_->value()),
    rclcpp::Parameter("anytime_rrt_star.time_budget_ms", anytime_time_budget_->value()),
    rclcpp::Parameter("anytime_rrt_star.step_size", anytime_step_size_->value()),
    rclcpp::Parameter("anytime_rrt_star.rewire_radius", anytime_rewire_radius_->value()),
    rclcpp::Parameter("anytime_rrt_star.goal_bias", anytime_goal_bias_->value()),
    rclcpp::Parameter("anytime_rrt_star.seed", anytime_seed_->value()),
    rclcpp::Parameter("informed_rrt_star.max_iterations", informed_iterations_->value()),
    rclcpp::Parameter("informed_rrt_star.step_size", informed_step_size_->value()),
    rclcpp::Parameter("informed_rrt_star.rewire_radius", informed_rewire_radius_->value()),
    rclcpp::Parameter("informed_rrt_star.goal_bias", informed_goal_bias_->value()),
    rclcpp::Parameter("informed_rrt_star.seed", informed_seed_->value())};

  parameter_client_->set_parameters(parameters);
  status_->setText("全部参数已发送，正在使用当前终点重新规划…");
  Q_EMIT configChanged();
}

}  // namespace grid_path_searcher_2d

PLUGINLIB_EXPORT_CLASS(grid_path_searcher_2d::RrtStarPanel, rviz_common::Panel)
