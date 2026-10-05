/**
 * @file grid_search_astar.cpp
 * @brief Dijkstra、A* 与 TimeBreak A* 的二维栅格搜索实现。
 *
 * 文件职责：集中展示三种算法的共同主线——开放队列、取出当前节点、遍历
 * 八邻居、松弛代价、记录父节点、回溯路径。Dijkstra 和 A* 只在队列优先级
 * 上不同；TimeBreak A* 只额外增加截止时间检查。
 *
 * 包含的函数：searchDijkstra()、searchAStar()、searchWithPriorityQueue() 和
 * searchTimedAStar()。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace grid_path_searcher_2d
{
namespace
{

/** 开放队列元素；priority 越小，越先被取出。 */
struct OpenNode
{
  double priority;
  int grid_index;

  bool operator>(const OpenNode & other) const
  {
    return priority > other.priority;
  }
};

/** 上、下、左、右和四个对角方向。 */
constexpr std::array<std::array<int, 2>, 8> kDirections{{
  {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
  {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}
}};

}  // namespace

SearchResult GridSearcher2D::searchDijkstra(
  const GridIndex & start, const GridIndex & goal) const
{
  return searchWithPriorityQueue(start, goal, PriorityMode::kDijkstra, 0.0);
}

SearchResult GridSearcher2D::searchAStar(
  const GridIndex & start, const GridIndex & goal, const double heuristic_weight) const
{
  return searchWithPriorityQueue(start, goal, PriorityMode::kAStar, heuristic_weight);
}

/**
 * Dijkstra 与 A* 共用的搜索主循环。
 *
 * Dijkstra：priority = g，只考虑起点到当前格的真实代价。
 * A*：priority = g + h，再加上当前格到终点的估计代价。
 */
SearchResult GridSearcher2D::searchWithPriorityQueue(
  const GridIndex & start, const GridIndex & goal, const PriorityMode mode,
  const double heuristic_weight) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    heuristic_weight < 0.0)
  {
    return result;
  }

  const bool use_heuristic = mode == PriorityMode::kAStar;
  const std::size_t grid_count = map_.data.size();
  const int start_index = toLinear(start.x, start.y);
  const int goal_index = toLinear(goal.x, goal.y);

  // g_score：起点到每个格子的当前最小代价。
  // parent：最短路径中每个格子的上一个格子。
  std::vector<double> g_score(grid_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(grid_count, -1);
  std::vector<bool> closed(grid_count, false);
  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open_queue;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  const double start_priority =
    use_heuristic ? heuristic_weight * heuristic(start, goal) : 0.0;
  open_queue.push({start_priority, start_index});

  while (!open_queue.empty()) {
    // 第一步：取出当前优先级最小的格子。
    const int current_index = open_queue.top().grid_index;
    open_queue.pop();

    if (closed[static_cast<std::size_t>(current_index)]) {continue;}
    closed[static_cast<std::size_t>(current_index)] = true;

    const GridIndex current{
      current_index % static_cast<int>(map_.info.width),
      current_index / static_cast<int>(map_.info.width)};
    result.visited.push_back(current);

    if (current_index == goal_index) {break;}

    // 第二步：检查当前格子的八个邻居。
    for (const auto & direction : kDirections) {
      const GridIndex neighbor{
        current.x + direction[0],
        current.y + direction[1]};

      if (!traversable(neighbor.x, neighbor.y)) {continue;}

      const bool diagonal = direction[0] != 0 && direction[1] != 0;
      if (diagonal &&
        (!traversable(current.x + direction[0], current.y) ||
        !traversable(current.x, current.y + direction[1])))
      {
        continue;
      }

      const int neighbor_index = toLinear(neighbor.x, neighbor.y);
      if (closed[static_cast<std::size_t>(neighbor_index)]) {continue;}

      // 第三步：计算“经过 current 到达 neighbor”的新代价。
      const double move_cost = diagonal ? std::sqrt(2.0) : 1.0;
      const double new_g = g_score[static_cast<std::size_t>(current_index)] + move_cost;

      // 第四步：新路线更短时，更新代价和父节点，这一步叫松弛。
      if (new_g >= g_score[static_cast<std::size_t>(neighbor_index)]) {continue;}

      g_score[static_cast<std::size_t>(neighbor_index)] = new_g;
      parent[static_cast<std::size_t>(neighbor_index)] = current_index;

      const double h = use_heuristic ? heuristic_weight * heuristic(neighbor, goal) : 0.0;
      open_queue.push({new_g + h, neighbor_index});
    }
  }

  if (!closed[static_cast<std::size_t>(goal_index)]) {return result;}

  result.path = reconstructPath(parent, start_index, goal_index);
  result.cost = g_score[static_cast<std::size_t>(goal_index)] * map_.info.resolution;
  result.success = true;
  return result;
}

/** 与普通 A* 相同，只在每轮循环开始时增加超时判断。 */
SearchResult GridSearcher2D::searchTimedAStar(
  const GridIndex & start, const GridIndex & goal, const double time_limit_ms,
  const double heuristic_weight) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    time_limit_ms <= 0.0 || heuristic_weight < 0.0)
  {
    return result;
  }

  const auto begin_time = std::chrono::steady_clock::now();
  const auto time_budget = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(time_limit_ms));
  const auto deadline = begin_time + time_budget;

  const std::size_t grid_count = map_.data.size();
  const int start_index = toLinear(start.x, start.y);
  const int goal_index = toLinear(goal.x, goal.y);
  std::vector<double> g_score(grid_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(grid_count, -1);
  std::vector<bool> closed(grid_count, false);
  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open_queue;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open_queue.push({heuristic_weight * heuristic(start, goal), start_index});

  while (!open_queue.empty()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      result.timed_out = true;
      return result;
    }

    const int current_index = open_queue.top().grid_index;
    open_queue.pop();
    if (closed[static_cast<std::size_t>(current_index)]) {continue;}
    closed[static_cast<std::size_t>(current_index)] = true;

    const GridIndex current{
      current_index % static_cast<int>(map_.info.width),
      current_index / static_cast<int>(map_.info.width)};
    result.visited.push_back(current);
    if (current_index == goal_index) {break;}

    for (const auto & direction : kDirections) {
      const GridIndex neighbor{
        current.x + direction[0],
        current.y + direction[1]};
      if (!traversable(neighbor.x, neighbor.y)) {continue;}

      const bool diagonal = direction[0] != 0 && direction[1] != 0;
      if (diagonal &&
        (!traversable(current.x + direction[0], current.y) ||
        !traversable(current.x, current.y + direction[1])))
      {
        continue;
      }

      const int neighbor_index = toLinear(neighbor.x, neighbor.y);
      if (closed[static_cast<std::size_t>(neighbor_index)]) {continue;}

      const double move_cost = diagonal ? std::sqrt(2.0) : 1.0;
      const double new_g = g_score[static_cast<std::size_t>(current_index)] + move_cost;
      if (new_g >= g_score[static_cast<std::size_t>(neighbor_index)]) {continue;}

      g_score[static_cast<std::size_t>(neighbor_index)] = new_g;
      parent[static_cast<std::size_t>(neighbor_index)] = current_index;
      open_queue.push({new_g + heuristic_weight * heuristic(neighbor, goal), neighbor_index});
    }
  }

  if (!closed[static_cast<std::size_t>(goal_index)]) {return result;}

  result.path = reconstructPath(parent, start_index, goal_index);
  result.cost = g_score[static_cast<std::size_t>(goal_index)] * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
