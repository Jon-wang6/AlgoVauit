/**
 * @file grid_search_rrt.cpp
 * @brief 二维 RRT 快速随机树算法实现。
 *
 * 文件职责：从起点建立一棵树，反复执行随机采样、寻找最近节点、向样本扩展
 * 和碰撞检测；树能连接终点时，通过父节点回溯出一条可行路径。
 *
 * 包含的函数：searchRrt()。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace grid_path_searcher_2d
{

SearchResult GridSearcher2D::searchRrt(
  const GridIndex & start, const GridIndex & goal, const int max_iterations,
  const double step_size, const double goal_bias, const int seed) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    max_iterations <= 0 || step_size <= 0.0)
  {
    return result;
  }

  if (start.x == goal.x && start.y == goal.y) {
    result.success = true;
    result.path.push_back(start);
    return result;
  }

  // nodes 保存树节点，parent[i] 保存节点 i 的父节点下标。
  std::vector<GridIndex> nodes{start};
  std::vector<int> parent{-1};
  std::vector<bool> inserted(map_.data.size(), false);
  inserted[static_cast<std::size_t>(toLinear(start.x, start.y))] = true;
  result.visited.push_back(start);

  std::mt19937 random_generator(static_cast<std::mt19937::result_type>(seed));
  std::uniform_int_distribution<int> random_x(0, static_cast<int>(map_.info.width) - 1);
  std::uniform_int_distribution<int> random_y(0, static_cast<int>(map_.info.height) - 1);
  std::uniform_real_distribution<double> random_probability(0.0, 1.0);
  const double safe_goal_bias = std::clamp(goal_bias, 0.0, 1.0);
  int goal_node_index = -1;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    // 第一步：以 goal_bias 概率直接选终点，否则随机选一个自由格。
    GridIndex random_sample;
    if (random_probability(random_generator) < safe_goal_bias) {
      random_sample = goal;
    } else {
      random_sample = {random_x(random_generator), random_y(random_generator)};
      if (!traversable(random_sample.x, random_sample.y)) {continue;}
    }

    // 第二步：找树中距离随机样本最近的节点。
    int nearest_node_index = 0;
    double shortest_squared_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double dx = static_cast<double>(random_sample.x - nodes[index].x);
      const double dy = static_cast<double>(random_sample.y - nodes[index].y);
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < shortest_squared_distance) {
        shortest_squared_distance = squared_distance;
        nearest_node_index = static_cast<int>(index);
      }
    }

    // 第三步：从最近节点朝样本方向最多走 step_size 个栅格。
    const GridIndex & nearest_node = nodes[static_cast<std::size_t>(nearest_node_index)];
    const double delta_x = static_cast<double>(random_sample.x - nearest_node.x);
    const double delta_y = static_cast<double>(random_sample.y - nearest_node.y);
    const double distance_to_sample = std::hypot(delta_x, delta_y);
    if (distance_to_sample < 1e-9) {continue;}

    const double travel_distance = std::min(step_size, distance_to_sample);
    const GridIndex new_node{
      static_cast<int>(std::lround(
        nearest_node.x + travel_distance * delta_x / distance_to_sample)),
      static_cast<int>(std::lround(
        nearest_node.y + travel_distance * delta_y / distance_to_sample))};

    // 第四步：新节点和扩展边必须无碰撞，而且该格不能已经在树中。
    if (!traversable(new_node.x, new_node.y)) {continue;}
    const auto new_grid_index = static_cast<std::size_t>(toLinear(new_node.x, new_node.y));
    if (inserted[new_grid_index] || !lineTraversable(nearest_node, new_node)) {continue;}

    inserted[new_grid_index] = true;
    const int new_node_index = static_cast<int>(nodes.size());
    nodes.push_back(new_node);
    parent.push_back(nearest_node_index);
    result.visited.push_back(new_node);
    result.tree_edges.push_back({nearest_node, new_node});

    if (new_node.x == goal.x && new_node.y == goal.y) {
      goal_node_index = new_node_index;
      break;
    }

    // 第五步：新节点距离终点足够近且能直连时，把终点加入树并停止。
    const double goal_dx = static_cast<double>(goal.x - new_node.x);
    const double goal_dy = static_cast<double>(goal.y - new_node.y);
    if (std::hypot(goal_dx, goal_dy) <= step_size && lineTraversable(new_node, goal)) {
      goal_node_index = static_cast<int>(nodes.size());
      nodes.push_back(goal);
      parent.push_back(new_node_index);
      result.visited.push_back(goal);
      result.tree_edges.push_back({new_node, goal});
      break;
    }
  }

  if (goal_node_index < 0) {return result;}

  // 第六步：从终点沿父节点回溯，然后反转为起点到终点。
  for (int current = goal_node_index; current >= 0;
    current = parent[static_cast<std::size_t>(current)])
  {
    result.path.push_back(nodes[static_cast<std::size_t>(current)]);
    if (current == 0) {break;}
  }
  std::reverse(result.path.begin(), result.path.end());

  double path_length_in_cells = 0.0;
  for (std::size_t index = 1; index < result.path.size(); ++index) {
    const double dx = static_cast<double>(result.path[index].x - result.path[index - 1].x);
    const double dy = static_cast<double>(result.path[index].y - result.path[index - 1].y);
    path_length_in_cells += std::hypot(dx, dy);
  }

  result.cost = path_length_in_cells * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
