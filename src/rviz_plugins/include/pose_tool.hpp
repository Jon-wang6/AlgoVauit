/**
 * @file pose_tool.hpp
 * @brief RViz2 三维目标交互工具的抽象基类声明。
 *
 * 文件职责：定义鼠标交互状态机和箭头可视化资源；用户依次选择位置、
 * 航向与高度后，通过纯虚函数 onPoseSet() 把结果交给派生类。
 *
 * 包含的类：
 * - Pose3DTool：继承 rviz_common::Tool 的三阶段交互基类。
 *
 * 包含的函数：构造/析构函数、onInitialize()、activate()、deactivate()、
 * processMouseEvent()、onPoseSet()、clearHeightArrows() 和
 * setMainArrowOrientation()。
 */
#ifndef RVIZ_PLUGINS__POSE_TOOL_HPP_
#define RVIZ_PLUGINS__POSE_TOOL_HPP_

#include <memory>
#include <vector>

#include <OgreVector3.h>
#include "rviz_common/tool.hpp"

namespace rviz_rendering
{
class Arrow;
class ViewportProjectionFinder;
}

namespace rviz_plugins
{
/**
 * RViz 三维姿态交互工具基类。
 * 鼠标状态依次为位置、朝向和高度；完成后调用派生类 onPoseSet。
 */
class Pose3DTool : public rviz_common::Tool
{
public:
  Pose3DTool();
  ~Pose3DTool() override;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  /** 派生类实现最终姿态的处理方式，本项目用于发布 /goal。 */
  virtual void onPoseSet(double x, double y, double z, double yaw) = 0;

private:
  // 一次鼠标交互的三个阶段。
  enum class State {Position, Orientation, Height};
  void clearHeightArrows();
  void setMainArrowOrientation(double yaw);

  State state_{State::Position};
  Ogre::Vector3 position_{Ogre::Vector3::ZERO};
  double initial_z_{0.0};
  double previous_mouse_y_{0.0};
  double yaw_{0.0};
  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> height_arrows_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
};
}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS__POSE_TOOL_HPP_
