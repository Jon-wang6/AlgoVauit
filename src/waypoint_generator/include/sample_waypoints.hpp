/**
 * @file sample_waypoints.hpp
 * @brief 可复用的示例航点轨迹生成函数集合（仅头文件实现）。
 *
 * 文件职责：生成姿态四元数，并把坐标点组装成 nav_msgs/Path；提供教程
 * 使用的折线、圆形和“8”字形轨迹，无需额外 .cpp 编译单元。
 *
 * 包含的类：无。
 * 包含的函数：
 * - yawQuaternion()：把 yaw 转换成 geometry_msgs 四元数。
 * - fromPoints()：把 {x,y,z} 点集转换成 Path。
 * - point()：生成折线轨迹。
 * - circle()：生成圆形/回环轨迹。
 * - eight()：生成“8”字形轨迹。
 */
#ifndef WAYPOINT_GENERATOR__SAMPLE_WAYPOINTS_HPP_
#define WAYPOINT_GENERATOR__SAMPLE_WAYPOINTS_HPP_

#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace waypoint_patterns
{
/** 根据偏航角生成只绕 Z 轴旋转的单位四元数。 */
inline geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

/** 将一组 {x,y,z} 数组转换成 nav_msgs/Path，默认航向为 0。 */
inline nav_msgs::msg::Path fromPoints(const std::vector<std::vector<double>> & points)
{
  nav_msgs::msg::Path path;
  for (const auto & xyz : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = xyz[0];
    pose.pose.position.y = xyz[1];
    pose.pose.position.z = xyz[2];
    pose.pose.orientation = yawQuaternion(0.0);
    path.poses.push_back(pose);
  }
  return path;
}

/** 生成教程中的折线路径预设。 */
inline nav_msgs::msg::Path point()
{
  constexpr double s = 7.0;
  constexpr double h = 1.0;
  return fromPoints({
    {2.0 * s, 0.0, h}, {4.0 * s, 0.0, h}, {5.0 * s, 0.25 * s, h},
    {5.3 * s, 0.5 * s, h}, {5.0 * s, 0.75 * s, h}, {4.0 * s, s, h},
    {2.0 * s, s, h}, {0.0, s, h}});
}

/** 生成由两圈控制点组成的圆形/回环路径预设。 */
inline nav_msgs::msg::Path circle()
{
  constexpr double s = 5.0;
  constexpr double h = 1.0;
  std::vector<std::vector<double>> points;
  for (int repeat = 0; repeat < 2; ++repeat) {
    points.insert(points.end(), {
      {2.5 * s, -1.2 * s, h}, {5.0 * s, -2.4 * s, h}, {5.0 * s, 0.0, h},
      {2.5 * s, -1.2 * s, h}, {0.0, -2.4 * s, h}, {0.0, 0.0, h}});
  }
  return fromPoints(points);
}

/** 生成在平面与高度方向交叉的三维八字形路径预设。 */
inline nav_msgs::msg::Path eight()
{
  constexpr double r = 10.0;
  constexpr double h = 2.0;
  return fromPoints({
    {r, -r, h / 2.0}, {2 * r, 0, h}, {3 * r, r, h / 2.0}, {4 * r, 0, h},
    {3 * r, -r, h / 2.0}, {2 * r, 0, h}, {r, r, h / 2.0}, {0, 0, h},
    {r, -r, 3 * h / 2.0}, {2 * r, 0, h}, {3 * r, r, 3 * h / 2.0}, {4 * r, 0, h},
    {3 * r, -r, 3 * h / 2.0}, {2 * r, 0, h}, {r, r, 3 * h / 2.0}, {0, 0, h}});
}
}  // namespace waypoint_patterns

#endif  // WAYPOINT_GENERATOR__SAMPLE_WAYPOINTS_HPP_
