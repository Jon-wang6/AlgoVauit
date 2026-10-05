/**
 * @file grid_search_rrt_star.cpp
 * @brief 二维 RRT* 优化随机树算法实现。
 *
 * 文件职责：先像 RRT 一样采样和扩展，再为新节点选择总代价最低的父节点，
 * 并尝试重连附近节点。重连会让搜索树中的已有路径持续变短。
 *
 * 包含的函数：searchRrtStar()。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace grid_path_searcher_2d
{
namespace
{

/** 重连半径内一个可直连的已有树节点。 */
struct NearbyNode
{
  int node_index;
  double distance;
};

}  // namespace

SearchResult GridSearcher2D::searchRrtStar(
  const GridIndex & start, const GridIndex & goal, const int max_iterations,
  const double step_size, const double rewire_radius, const double goal_bias,
  const int seed) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    max_iterations <= 0 || step_size <= 0.0 || rewire_radius <= 0.0)
  {
    return result;
  }

  if (start.x == goal.x && start.y == goal.y) {
    result.success = true;
    result.path.push_back(start);
    return result;
  }

  // RRT* 除了节点和父节点，还要记录起点代价与子节点，方便重连和代价传播。
  std::vector<GridIndex> nodes{start};
  std::vector<int> parent{-1};
  std::vector<double> cost_from_start{0.0};
  std::vector<std::vector<int>> children(1);
  std::vector<bool> inserted(map_.data.size(), false);
  inserted[static_cast<std::size_t>(toLinear(start.x, start.y))] = true;
  result.visited.push_back(start);

  std::mt19937 random_generator(static_cast<std::mt19937::result_type>(seed));
  std::uniform_int_distribution<int> random_x(0, static_cast<int>(map_.info.width) - 1);
  std::uniform_int_distribution<int> random_y(0, static_cast<int>(map_.info.height) - 1);
  std::uniform_real_distribution<double> random_probability(0.0, 1.0);
  const double safe_goal_bias = std::clamp(goal_bias, 0.0, 1.0);

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    // 一、Sample：随机采样，偶尔直接采样终点。
    GridIndex random_sample;
    if (random_probability(random_generator) < safe_goal_bias) {
      random_sample = goal;
    } else {
      random_sample = {random_x(random_generator), random_y(random_generator)};
      if (!traversable(random_sample.x, random_sample.y)) {continue;}
    }

    // 二、Nearest：寻找距离样本最近的树节点。
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

    // 三、Steer：从最近节点向样本最多前进 step_size 个栅格。
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

    if (!traversable(new_node.x, new_node.y)) {continue;}
    if (new_node.x == goal.x && new_node.y == goal.y) {continue;}
    const auto new_grid_index = static_cast<std::size_t>(toLinear(new_node.x, new_node.y));
    if (inserted[new_grid_index] || !lineTraversable(nearest_node, new_node)) {continue;}

    // 四、Near：收集重连半径内且能无碰撞直连的已有节点。
    std::vector<NearbyNode> nearby_nodes;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double dx = static_cast<double>(new_node.x - nodes[index].x);
      const double dy = static_cast<double>(new_node.y - nodes[index].y);
      const double distance = std::hypot(dx, dy);
      if (distance <= rewire_radius && lineTraversable(nodes[index], new_node)) {
        nearby_nodes.push_back({static_cast<int>(index), distance});
      }
    }

    // 五、Choose parent：选择“起点总代价 + 新边长度”最小的父节点。
    int best_parent_index = nearest_node_index;
    double best_cost =
      cost_from_start[static_cast<std::size_t>(nearest_node_index)] +
      std::hypot(delta_x, delta_y);

    for (const auto & nearby : nearby_nodes) {
      const double candidate_cost =
        cost_from_start[static_cast<std::size_t>(nearby.node_index)] + nearby.distance;
      if (candidate_cost < best_cost) {
        best_parent_index = nearby.node_index;
        best_cost = candidate_cost;
      }
    }

    inserted[new_grid_index] = true;
    const int new_node_index = static_cast<int>(nodes.size());
    nodes.push_back(new_node);
    parent.push_back(best_parent_index);
    cost_from_start.push_back(best_cost);
    children.emplace_back();
    children[static_cast<std::size_t>(best_parent_index)].push_back(new_node_index);
    result.visited.push_back(new_node);

    // 六、Rewire：附近节点经过新节点更短时，更换父节点。
    for (const auto & nearby : nearby_nodes) {
      const int neighbor_index = nearby.node_index;
      if (neighbor_index == best_parent_index || neighbor_index == 0) {continue;}

      const double rewired_cost = best_cost + nearby.distance;
      if (rewired_cost + 1e-9 >=
        cost_from_start[static_cast<std::size_t>(neighbor_index)])
      {
        continue;
      }

      const int old_parent_index = parent[static_cast<std::size_t>(neighbor_index)];
      if (old_parent_index >= 0) {
        auto & old_children = children[static_cast<std::size_t>(old_parent_index)];
        old_children.erase(
          std::remove(old_children.begin(), old_children.end(), neighbor_index),
          old_children.end());
      }

      parent[static_cast<std::size_t>(neighbor_index)] = new_node_index;
      children[static_cast<std::size_t>(new_node_index)].push_back(neighbor_index);

      // 父节点变化后，整棵子树的累计代价都要加上同一个差值。
      const double cost_change =
        rewired_cost - cost_from_start[static_cast<std::size_t>(neighbor_index)];
      std::vector<int> update_stack{neighbor_index};
      while (!update_stack.empty()) {
        const int current = update_stack.back();
        update_stack.pop_back();
        cost_from_start[static_cast<std::size_t>(current)] += cost_change;
        for (const int child : children[static_cast<std::size_t>(current)]) {
          update_stack.push_back(child);
        }
      }
    }
  }

  // 根据最终父节点关系生成 RViz 中显示的重连后搜索树。
  for (std::size_t index = 1; index < nodes.size(); ++index) {
    const int parent_index = parent[index];
    if (parent_index >= 0) {
      result.tree_edges.push_back({
        nodes[static_cast<std::size_t>(parent_index)], nodes[index]});
    }
  }

  // 迭代结束后，从能直连终点的节点中选总代价最低者。
  int best_goal_parent_index = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double dx = static_cast<double>(goal.x - nodes[index].x);
    const double dy = static_cast<double>(goal.y - nodes[index].y);
    const double distance_to_goal = std::hypot(dx, dy);

    if (distance_to_goal > step_size || !lineTraversable(nodes[index], goal)) {continue;}

    const double candidate_cost = cost_from_start[index] + distance_to_goal;
    if (candidate_cost < best_goal_cost) {
      best_goal_cost = candidate_cost;
      best_goal_parent_index = static_cast<int>(index);
    }
  }

  if (best_goal_parent_index < 0) {return result;}

  result.tree_edges.push_back({nodes[static_cast<std::size_t>(best_goal_parent_index)], goal});
  result.path.push_back(goal);
  for (int current = best_goal_parent_index; current >= 0;
    current = parent[static_cast<std::size_t>(current)])
  {
    result.path.push_back(nodes[static_cast<std::size_t>(current)]);
    if (current == 0) {break;}
  }
  std::reverse(result.path.begin(), result.path.end());

  result.cost = best_goal_cost * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
