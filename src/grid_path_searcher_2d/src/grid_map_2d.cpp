/**
 * @file grid_map_2d.cpp
 * @brief 二维规划器共用的地图、坐标和路径工具。
 *
 * 文件职责：保存 OccupancyGrid，判断栅格能否通行，完成世界坐标与栅格坐标
 * 转换，并提供父节点回溯和直线碰撞检测。这里不实现具体搜索算法，读算法时
 * 可以暂时把这些函数当作已经准备好的基础工具。
 *
 * 包含的函数：setMap()、worldToGrid()、gridToWorld()、inBounds()、toLinear()、
 * traversable()、heuristic()、reconstructPath() 和 lineTraversable()。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace grid_path_searcher_2d
{

bool GridSearcher2D::setMap(
  const nav_msgs::msg::OccupancyGrid & map, const int occupied_threshold,
  const bool allow_unknown)
{
  const auto expected_size = static_cast<std::size_t>(map.info.width) * map.info.height;
  ready_ = map.info.resolution > 0.0F && map.info.width > 0U && map.info.height > 0U &&
    map.data.size() == expected_size;
  if (!ready_) {return false;}

  map_ = map;
  occupied_threshold_ = occupied_threshold;
  allow_unknown_ = allow_unknown;
  return true;
}

bool GridSearcher2D::worldToGrid(
  const double world_x, const double world_y, GridIndex & index) const
{
  if (!ready_) {return false;}

  const auto & origin = map_.info.origin;
  tf2::Quaternion quaternion;
  tf2::fromMsg(origin.orientation, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  // 先减去地图原点，再按地图朝向的反方向旋转到地图局部坐标系。
  const double offset_x = world_x - origin.position.x;
  const double offset_y = world_y - origin.position.y;
  const double local_x = std::cos(yaw) * offset_x + std::sin(yaw) * offset_y;
  const double local_y = -std::sin(yaw) * offset_x + std::cos(yaw) * offset_y;

  index.x = static_cast<int>(std::floor(local_x / map_.info.resolution));
  index.y = static_cast<int>(std::floor(local_y / map_.info.resolution));
  return inBounds(index.x, index.y);
}

geometry_msgs::msg::Point GridSearcher2D::gridToWorld(const GridIndex & index) const
{
  const auto & origin = map_.info.origin;
  tf2::Quaternion quaternion;
  tf2::fromMsg(origin.orientation, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  // 加 0.5 是为了取得栅格中心，而不是左下角。
  const double local_x = (static_cast<double>(index.x) + 0.5) * map_.info.resolution;
  const double local_y = (static_cast<double>(index.y) + 0.5) * map_.info.resolution;

  geometry_msgs::msg::Point point;
  point.x = origin.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y;
  point.y = origin.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y;
  point.z = origin.position.z;
  return point;
}

bool GridSearcher2D::inBounds(const int x, const int y) const
{
  return ready_ && x >= 0 && y >= 0 && x < static_cast<int>(map_.info.width) &&
         y < static_cast<int>(map_.info.height);
}

int GridSearcher2D::toLinear(const int x, const int y) const
{
  return y * static_cast<int>(map_.info.width) + x;
}

bool GridSearcher2D::traversable(const int x, const int y) const
{
  if (!inBounds(x, y)) {return false;}

  const auto occupancy = map_.data[static_cast<std::size_t>(toLinear(x, y))];
  if (occupancy < 0) {return allow_unknown_;}
  return occupancy < occupied_threshold_;
}

double GridSearcher2D::heuristic(const GridIndex & from, const GridIndex & to)
{
  const double delta_x = std::abs(from.x - to.x);
  const double delta_y = std::abs(from.y - to.y);

  // 八角距离：斜走 min(dx,dy) 次，再直走剩余距离。
  return std::max(delta_x, delta_y) +
         (std::sqrt(2.0) - 1.0) * std::min(delta_x, delta_y);
}

std::vector<GridIndex> GridSearcher2D::reconstructPath(
  const std::vector<int> & parent, const int start_linear, const int goal_linear) const
{
  std::vector<GridIndex> path;

  // parent 记录“当前格从哪个格走来”，所以先从终点反向追到起点。
  for (int current = goal_linear; current >= 0;
    current = parent[static_cast<std::size_t>(current)])
  {
    path.push_back({
      current % static_cast<int>(map_.info.width),
      current / static_cast<int>(map_.info.width)});
    if (current == start_linear) {break;}
  }

  std::reverse(path.begin(), path.end());
  return path;
}

bool GridSearcher2D::lineTraversable(
  const GridIndex & from, const GridIndex & to) const
{
  // Bresenham 算法沿线段逐格前进，遇到一个障碍就判定这条边不可用。
  int x = from.x;
  int y = from.y;
  const int delta_x = std::abs(to.x - from.x);
  const int delta_y = std::abs(to.y - from.y);
  const int step_x = from.x < to.x ? 1 : -1;
  const int step_y = from.y < to.y ? 1 : -1;
  int error = delta_x - delta_y;

  while (true) {
    if (!traversable(x, y)) {return false;}
    if (x == to.x && y == to.y) {return true;}

    const int previous_x = x;
    const int previous_y = y;
    const int twice_error = 2 * error;

    if (twice_error > -delta_y) {
      error -= delta_y;
      x += step_x;
    }
    if (twice_error < delta_x) {
      error += delta_x;
      y += step_y;
    }

    // 同时改变 x/y 代表斜向移动；两个侧格必须都能通过，禁止穿墙角。
    if (x != previous_x && y != previous_y &&
      (!traversable(x, previous_y) || !traversable(previous_x, y)))
    {
      return false;
    }
  }
}

}  // namespace grid_path_searcher_2d
