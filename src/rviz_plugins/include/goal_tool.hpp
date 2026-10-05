/**
 * @file goal_tool.hpp
 * @brief 将 Pose3DTool 交互结果发布为 ROS 2 三维目标的派生类声明。
 *
 * 文件职责：定义 RViz 属性面板中的目标话题配置，并保存 RViz ROS 节点和
 * PoseStamped 发布器。
 *
 * 包含的类：
 * - Goal3DTool：Pose3DTool 的发布实现，也是 pluginlib 注册的实际工具类。
 *
 * 包含的函数：Goal3DTool()、onInitialize()、onPoseSet()、updateTopic()。
 */
#ifndef RVIZ_PLUGINS__GOAL_TOOL_HPP_
#define RVIZ_PLUGINS__GOAL_TOOL_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pose_tool.hpp"

namespace rviz_common {namespace properties {class StringProperty;}}

namespace rviz_plugins
{
/** Pose3DTool 的 ROS 2 发布实现：把最终姿态封装为 PoseStamped。 */
class Goal3DTool : public Pose3DTool
{
  Q_OBJECT
public:
  Goal3DTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double yaw) override;

private Q_SLOTS:
  /** RViz 属性面板中的 Topic 改变时重建发布器。 */
  void updateTopic();

private:
  rviz_common::properties::StringProperty * topic_property_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};
}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS__GOAL_TOOL_HPP_
