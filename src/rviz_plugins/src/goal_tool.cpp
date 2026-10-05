/**
 * @file goal_tool.cpp
 * @brief Goal3DTool 的 ROS 2 发布逻辑与 pluginlib 导出实现。
 *
 * 文件职责：创建可编辑的 Topic 属性，取得 RViz 内部 ROS 节点，将三维交互
 * 结果转换为 PoseStamped，并导出为 RViz2 可加载插件。
 *
 * 包含的类：Goal3DTool（声明位于 include/goal_tool.hpp）。
 * 包含的函数：Goal3DTool()、onInitialize()、updateTopic()、onPoseSet()；
 * 文件末尾使用 PLUGINLIB_EXPORT_CLASS 注册插件类。
 */
#include "goal_tool.hpp"

#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rviz_plugins
{
/** 在 RViz 属性面板创建可编辑的目标话题属性，默认使用 /goal。 */
Goal3DTool::Goal3DTool()
{
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/goal", "Topic used to publish 3D navigation goals.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

/** 初始化基类图形对象，获取 RViz 内部 ROS 节点并创建目标发布器。 */
void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {throw std::runtime_error("RViz ROS node is unavailable");}
  node_ = abstraction->get_raw_node();
  updateTopic();
}

/** 使用属性面板中的最新话题名重建 PoseStamped 发布器。 */
void Goal3DTool::updateTopic()
{
  if (!node_) {return;}
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::QoS(1).reliable());
}

/**
 * 将交互结果转换为 PoseStamped：位置直接写入 XYZ，yaw 转为四元数，
 * 坐标系采用 RViz Fixed Frame，随后发布并打印目标信息。
 */
void Goal3DTool::onPoseSet(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.header.stamp = node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  goal.pose.orientation = tf2::toMsg(quaternion);
  publisher_->publish(goal);
  RCLCPP_INFO(node_->get_logger(), "3D goal: frame=%s xyz=(%.3f, %.3f, %.3f) yaw=%.3f",
    goal.header.frame_id.c_str(), x, y, z, yaw);
}
}  // namespace rviz_plugins

// 向 pluginlib 注册类，使 RViz2 能在工具列表中动态加载 Goal3DTool。
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Goal3DTool, rviz_common::Tool)
