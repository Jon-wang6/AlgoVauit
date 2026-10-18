/**
 * @file grid_search_theta_star.cpp
 * @brief Theta* 任意角度栅格搜索实现。
 *
 * Theta* 保留 A* 的开放队列和八邻域扩展，但更新邻居时会检查当前节点的
 * 父节点能否直接看见该邻居。若视线无碰撞，邻居直接继承祖父节点，从而
 * 跳过不必要的栅格折点；若视线被挡住，则退回普通相邻节点松弛。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace grid_path_searcher_2d
{
namespace
{

struct ThetaOpenNode
{
  double priority;
  int grid_index;

  bool operator>(const ThetaOpenNode & other) const
  {
    return priority > other.priority;
  }
};

constexpr std::array<std::array<int, 2>, 8> kThetaDirections{{
  {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
  {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}
}};

double euclideanDistance(const GridIndex & from, const GridIndex & to)
{
  return std::hypot(
    static_cast<double>(to.x - from.x),
    static_cast<double>(to.y - from.y));
}

}  // namespace

SearchResult GridSearcher2D::searchThetaStar(
  const GridIndex & start, const GridIndex & goal,
  const double heuristic_weight) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    heuristic_weight < 0.0)
  {
    return result;
  }

  const std::size_t grid_count = map_.data.size();
  const int width = static_cast<int>(map_.info.width);
  const int start_index = toLinear(start.x, start.y);
  const int goal_index = toLinear(goal.x, goal.y);

  std::vector<double> g_score(grid_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(grid_count, -1);
  std::vector<bool> closed(grid_count, false);
  std::priority_queue<
    ThetaOpenNode, std::vector<ThetaOpenNode>, std::greater<ThetaOpenNode>> open_queue;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  // 起点以自己为父节点，第一次扩展也可以统一使用父节点视线检查逻辑。
  parent[static_cast<std::size_t>(start_index)] = start_index;
  open_queue.push({heuristic_weight * euclideanDistance(start, goal), start_index});

  while (!open_queue.empty()) {
    const int current_index = open_queue.top().grid_index;
    open_queue.pop();
    if (closed[static_cast<std::size_t>(current_index)]) {continue;}
    closed[static_cast<std::size_t>(current_index)] = true;

    const GridIndex current{current_index % width, current_index / width};
    result.visited.push_back(current);
    if (current_index == goal_index) {break;}

    const int current_parent_index = parent[static_cast<std::size_t>(current_index)];
    const GridIndex current_parent{
      current_parent_index % width, current_parent_index / width};

    for (const auto & direction : kThetaDirections) {
      const GridIndex neighbor{
        current.x + direction[0], current.y + direction[1]};
      if (!traversable(neighbor.x, neighbor.y)) {continue;}

      // 与 A* 保持相同的防穿墙角规则。
      const bool diagonal = direction[0] != 0 && direction[1] != 0;
      if (diagonal &&
        (!traversable(current.x + direction[0], current.y) ||
        !traversable(current.x, current.y + direction[1])))
      {
        continue;
      }

      const int neighbor_index = toLinear(neighbor.x, neighbor.y);
      if (closed[static_cast<std::size_t>(neighbor_index)]) {continue;}

      int candidate_parent = current_index;
      double candidate_g =
        g_score[static_cast<std::size_t>(current_index)] +
        euclideanDistance(current, neighbor);

      // Theta* 的核心：能看见祖父节点时直接继承它，消除中间折点。
      if (lineTraversable(current_parent, neighbor)) {
        const double shortcut_g =
          g_score[static_cast<std::size_t>(current_parent_index)] +
          euclideanDistance(current_parent, neighbor);
        if (shortcut_g < candidate_g) {
          candidate_g = shortcut_g;
          candidate_parent = current_parent_index;
        }
      }

      if (candidate_g >= g_score[static_cast<std::size_t>(neighbor_index)]) {continue;}

      g_score[static_cast<std::size_t>(neighbor_index)] = candidate_g;
      parent[static_cast<std::size_t>(neighbor_index)] = candidate_parent;
      const double h = heuristic_weight * euclideanDistance(neighbor, goal);
      open_queue.push({candidate_g + h, neighbor_index});
    }
  }

  if (!closed[static_cast<std::size_t>(goal_index)]) {return result;}

  result.path = reconstructPath(parent, start_index, goal_index);
  result.cost = g_score[static_cast<std::size_t>(goal_index)] * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
