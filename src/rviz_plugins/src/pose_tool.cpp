/**
 * @file pose_tool.cpp
 * @brief Pose3DTool 三阶段鼠标交互与 OGRE 箭头显示的实现。
 *
 * 文件职责：把 RViz 鼠标事件解释为三维位置、水平航向和高度，并维护主箭头
 * 与高度辅助箭头；交互完成后调用派生类的 onPoseSet()。
 *
 * 包含的类：Pose3DTool（声明位于 include/pose_tool.hpp）。
 * 包含的函数：构造/析构函数、onInitialize()、activate()、deactivate()、
 * clearHeightArrows()、setMainArrowOrientation()、processMouseEvent()。
 */
#include "pose_tool.hpp"

#include <cmath>
#include <memory>

#include <OgreMath.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <QEvent>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace rviz_plugins
{
/** 设置快捷键 g。 */
Pose3DTool::Pose3DTool() {shortcut_key_ = 'g';}
/** 释放交互过程中临时创建的高度箭头。 */
Pose3DTool::~Pose3DTool() {clearHeightArrows();}

/** 创建屏幕投影辅助器和主方向箭头，初始保持隐藏。 */
void Pose3DTool::onInitialize()
{
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 2.0F, 0.2F, 0.5F, 0.35F);
  arrow_->setColor(0.0F, 1.0F, 0.0F, 1.0F);
  arrow_->getSceneNode()->setVisible(false);
}

/** 激活工具时显示操作提示，并从“选择位置”阶段开始。 */
void Pose3DTool::activate()
{
  setStatus("Left-drag sets position/yaw; hold right button while dragging vertically to set height.");
  state_ = State::Position;
}

/** 切换到其他工具时隐藏箭头并清除临时高度标记。 */
void Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
  clearHeightArrows();
}

/** 清除表示高度差的辅助箭头。 */
void Pose3DTool::clearHeightArrows() {height_arrows_.clear();}

/** 把主箭头从模型默认方向旋转到用户拖出的 yaw 方向。 */
void Pose3DTool::setMainArrowOrientation(double yaw)
{
  const Ogre::Quaternion point_along_x(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
  arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z) * point_along_x);
}

/**
 * 处理完整鼠标状态机：
 * 1. 左键按下：将屏幕坐标投影到 XY 平面并确定位置；
 * 2. 左键拖动：根据当前位置到鼠标投影点计算 yaw；
 * 3. 同时按右键上下拖动：把屏幕位移换算为 Z 高度并绘制辅助箭头；
 * 4. 左键松开：调用 onPoseSet 发布最终目标并结束工具操作。
 */
int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;
  if (event.leftDown()) {
    const auto hit = projection_finder_->getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
    if (hit.first) {
      position_ = hit.second;
      initial_z_ = position_.z;
      previous_mouse_y_ = event.y;
      arrow_->setPosition(position_);
      state_ = State::Orientation;
      flags |= Render;
    }
  } else if (event.type == QEvent::MouseMove && event.left()) {
    if (state_ == State::Orientation) {
      const auto hit = projection_finder_->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
      if (hit.first) {
        yaw_ = std::atan2(hit.second.y - position_.y, hit.second.x - position_.x);
        setMainArrowOrientation(yaw_);
        arrow_->getSceneNode()->setVisible(true);
        if (event.right()) {
          state_ = State::Height;
          previous_mouse_y_ = event.y;
        }
        flags |= Render;
      }
    } else if (state_ == State::Height) {
      position_.z -= (event.y - previous_mouse_y_) / 50.0;
      previous_mouse_y_ = event.y;
      arrow_->setPosition(position_);
      clearHeightArrows();
      const int count = static_cast<int>(std::ceil(std::abs(initial_z_ - position_.z) / 0.5));
      for (int i = 0; i < count; ++i) {
        auto height_arrow = std::make_shared<rviz_rendering::Arrow>(
          scene_manager_, nullptr, 0.5F, 0.1F, 0.0F, 0.1F);
        height_arrow->setColor(0.0F, 1.0F, 0.0F, 1.0F);
        Ogre::Vector3 p = position_;
        p.z = initial_z_ - ((initial_z_ - position_.z > 0.0) ? 1.0 : -1.0) * i * 0.5;
        height_arrow->setPosition(p);
        height_arrow->setOrientation(arrow_->getOrientation());
        height_arrows_.push_back(height_arrow);
      }
      flags |= Render;
    }
  } else if (event.leftUp() && (state_ == State::Orientation || state_ == State::Height)) {
    clearHeightArrows();
    onPoseSet(position_.x, position_.y, position_.z, yaw_);
    flags |= Finished | Render;
  }
  return flags;
}
}  // namespace rviz_plugins
