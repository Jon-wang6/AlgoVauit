/**
 * @file grid_search_jps.cpp
 * @brief 二维 JPS 跳点搜索实现。
 *
 * 文件职责：判断强迫邻居，沿指定方向递归跳跃，并用 A* 风格的开放队列连接
 * 跳点。普通 A* 把相邻格加入队列，JPS 则跳过没有转折意义的普通格，只把
 * 目标或强迫邻居加入队列。
 *
 * 包含的函数：hasForcedNeighbor()、jump() 和 searchJps()。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace grid_path_searcher_2d
{
namespace
{

struct JpsOpenNode
{
  double priority;
  int grid_index;

  bool operator>(const JpsOpenNode & other) const
  {
    return priority > other.priority;
  }
};

constexpr std::array<std::array<int, 2>, 8> kDirections{{
  {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
  {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}
}};

}  // namespace

bool GridSearcher2D::hasForcedNeighbor(
  const GridIndex & cell, const int dx, const int dy) const
{
  if (dx != 0 && dy != 0) {
    // 斜向前进：侧后方被挡住，但对应侧前方可走。
    return
      (!traversable(cell.x - dx, cell.y) && traversable(cell.x - dx, cell.y + dy)) ||
      (!traversable(cell.x, cell.y - dy) && traversable(cell.x + dx, cell.y - dy));
  }

  if (dx != 0) {
    // 水平前进：检查上、下两侧。
    return
      (!traversable(cell.x - dx, cell.y + 1) && traversable(cell.x, cell.y + 1)) ||
      (!traversable(cell.x - dx, cell.y - 1) && traversable(cell.x, cell.y - 1));
  }

  // 垂直前进：检查左、右两侧。
  return
    (!traversable(cell.x + 1, cell.y - dy) && traversable(cell.x + 1, cell.y)) ||
    (!traversable(cell.x - 1, cell.y - dy) && traversable(cell.x - 1, cell.y));
}

bool GridSearcher2D::jump(
  const GridIndex & current, const int dx, const int dy, const GridIndex & goal,
  GridIndex & jump_point) const
{
  const GridIndex next{current.x + dx, current.y + dy};

  // 遇到地图边界或障碍，这个方向不能产生跳点。
  if (!traversable(next.x, next.y)) {return false;}

  // 斜向移动时禁止从两个障碍的墙角缝隙穿过。
  if (dx != 0 && dy != 0 &&
    (!traversable(current.x + dx, current.y) ||
    !traversable(current.x, current.y + dy)))
  {
    return false;
  }

  if ((next.x == goal.x && next.y == goal.y) || hasForcedNeighbor(next, dx, dy)) {
    jump_point = next;
    return true;
  }

  if (dx != 0 && dy != 0) {
    // 斜向跳跃时，只要水平或垂直分支发现跳点，当前格也必须保留。
    GridIndex branch_jump_point;
    if (jump(next, dx, 0, goal, branch_jump_point) ||
      jump(next, 0, dy, goal, branch_jump_point))
    {
      jump_point = next;
      return true;
    }
  }

  // 当前格没有特殊意义，继续沿同一方向向前跳。
  return jump(next, dx, dy, goal, jump_point);
}

SearchResult GridSearcher2D::searchJps(
  const GridIndex & start, const GridIndex & goal, const double heuristic_weight) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    heuristic_weight < 0.0)
  {
    return result;
  }

  const std::size_t grid_count = map_.data.size();
  const int start_index = toLinear(start.x, start.y);
  const int goal_index = toLinear(goal.x, goal.y);
  std::vector<double> g_score(grid_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(grid_count, -1);
  std::vector<bool> closed(grid_count, false);
  std::priority_queue<
    JpsOpenNode, std::vector<JpsOpenNode>, std::greater<JpsOpenNode>> open_queue;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open_queue.push({heuristic_weight * heuristic(start, goal), start_index});

  while (!open_queue.empty()) {
    const int current_index = open_queue.top().grid_index;
    open_queue.pop();

    if (closed[static_cast<std::size_t>(current_index)]) {continue;}
    closed[static_cast<std::size_t>(current_index)] = true;

    const GridIndex current{
      current_index % static_cast<int>(map_.info.width),
      current_index / static_cast<int>(map_.info.width)};
    result.visited.push_back(current);

    if (current_index == goal_index) {break;}

    // 与 A* 遍历邻居不同：这里从当前跳点向八个方向寻找“下一个跳点”。
    for (const auto & direction : kDirections) {
      GridIndex next_jump_point;
      if (!jump(current, direction[0], direction[1], goal, next_jump_point)) {
        continue;
      }

      const int next_index = toLinear(next_jump_point.x, next_jump_point.y);
      if (closed[static_cast<std::size_t>(next_index)]) {continue;}

      const double delta_x = static_cast<double>(next_jump_point.x - current.x);
      const double delta_y = static_cast<double>(next_jump_point.y - current.y);
      const double new_g =
        g_score[static_cast<std::size_t>(current_index)] + std::hypot(delta_x, delta_y);

      if (new_g >= g_score[static_cast<std::size_t>(next_index)]) {continue;}

      g_score[static_cast<std::size_t>(next_index)] = new_g;
      parent[static_cast<std::size_t>(next_index)] = current_index;
      open_queue.push({new_g + heuristic_weight * heuristic(next_jump_point, goal), next_index});
    }
  }

  if (!closed[static_cast<std::size_t>(goal_index)]) {return result;}

  result.jump_points = reconstructPath(parent, start_index, goal_index);
  result.path = result.jump_points;
  result.cost = g_score[static_cast<std::size_t>(goal_index)] * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
