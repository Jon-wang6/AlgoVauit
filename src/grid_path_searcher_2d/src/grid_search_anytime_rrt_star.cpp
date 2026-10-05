/**
 * @file grid_search_anytime_rrt_star.cpp
 * @brief 可在给定时间预算内持续改进解的 Anytime-RRT* 实现。
 *
 * 在普通 RRT* 的 Sample、Nearest、Steer、ChooseParent、Rewire 基础上，算法
 * 保存当前最优终点连接；首条路径出现后，用“起点到样本直线下界 + 样本到终点
 * 直线下界”剪掉不可能改善当前解的采样，并继续搜索直到时间或迭代预算耗尽。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace grid_path_searcher_2d
{
namespace
{

struct AnytimeNearbyNode
{
  int node_index;
  double distance;
};

}  // namespace

SearchResult GridSearcher2D::searchAnytimeRrtStar(
  const GridIndex & start, const GridIndex & goal, const int max_iterations,
  const double time_budget_ms, const double step_size, const double rewire_radius,
  const double goal_bias, const int seed) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    max_iterations <= 0 || time_budget_ms <= 0.0 || step_size <= 0.0 ||
    rewire_radius <= 0.0)
  {
    return result;
  }

  if (start.x == goal.x && start.y == goal.y) {
    result.success = true;
    result.path.push_back(start);
    return result;
  }

  const auto begin_time = std::chrono::steady_clock::now();
  const auto deadline = begin_time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(time_budget_ms));

  std::vector<GridIndex> nodes{start};
  std::vector<int> parent{-1};
  std::vector<double> cost_from_start{0.0};
  std::vector<std::vector<int>> children(1);
  std::vector<bool> inserted(map_.data.size(), false);
  inserted[static_cast<std::size_t>(toLinear(start.x, start.y))] = true;
  result.visited.push_back(start);

  std::mt19937 generator(static_cast<std::mt19937::result_type>(seed));
  std::uniform_int_distribution<int> random_x(0, static_cast<int>(map_.info.width) - 1);
  std::uniform_int_distribution<int> random_y(0, static_cast<int>(map_.info.height) - 1);
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  const double safe_goal_bias = std::clamp(goal_bias, 0.0, 1.0);
  double incumbent_cost = std::numeric_limits<double>::infinity();
  int incumbent_parent = -1;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      result.timed_out = true;
      break;
    }

    GridIndex random_sample;
    if (probability(generator) < safe_goal_bias) {
      random_sample = goal;
    } else {
      random_sample = {random_x(generator), random_y(generator)};
      if (!traversable(random_sample.x, random_sample.y)) {continue;}
    }

    // 首解之后进行分支限界：直线距离下界都不可能更优时无需扩展该样本。
    if (std::isfinite(incumbent_cost)) {
      const double lower_bound =
        std::hypot(
          static_cast<double>(random_sample.x - start.x),
          static_cast<double>(random_sample.y - start.y)) +
        std::hypot(
          static_cast<double>(goal.x - random_sample.x),
          static_cast<double>(goal.y - random_sample.y));
      if (lower_bound >= incumbent_cost) {continue;}
    }

    int nearest_index = 0;
    double nearest_squared_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double dx = static_cast<double>(random_sample.x - nodes[index].x);
      const double dy = static_cast<double>(random_sample.y - nodes[index].y);
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < nearest_squared_distance) {
        nearest_squared_distance = squared_distance;
        nearest_index = static_cast<int>(index);
      }
    }

    const GridIndex & nearest = nodes[static_cast<std::size_t>(nearest_index)];
    const double delta_x = static_cast<double>(random_sample.x - nearest.x);
    const double delta_y = static_cast<double>(random_sample.y - nearest.y);
    const double distance_to_sample = std::hypot(delta_x, delta_y);
    if (distance_to_sample < 1e-9) {continue;}

    const double travel_distance = std::min(step_size, distance_to_sample);
    const GridIndex new_node{
      static_cast<int>(std::lround(nearest.x + travel_distance * delta_x / distance_to_sample)),
      static_cast<int>(std::lround(nearest.y + travel_distance * delta_y / distance_to_sample))};
    if (!traversable(new_node.x, new_node.y) ||
      (new_node.x == goal.x && new_node.y == goal.y))
    {
      continue;
    }
    const auto new_grid_index = static_cast<std::size_t>(toLinear(new_node.x, new_node.y));
    if (inserted[new_grid_index] || !lineTraversable(nearest, new_node)) {continue;}

    std::vector<AnytimeNearbyNode> nearby_nodes;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double distance = std::hypot(
        static_cast<double>(new_node.x - nodes[index].x),
        static_cast<double>(new_node.y - nodes[index].y));
      if (distance <= rewire_radius && lineTraversable(nodes[index], new_node)) {
        nearby_nodes.push_back({static_cast<int>(index), distance});
      }
    }

    int best_parent = nearest_index;
    double best_cost = cost_from_start[static_cast<std::size_t>(nearest_index)] +
      std::hypot(delta_x, delta_y);
    for (const auto & nearby : nearby_nodes) {
      const double candidate_cost =
        cost_from_start[static_cast<std::size_t>(nearby.node_index)] + nearby.distance;
      if (candidate_cost < best_cost) {
        best_parent = nearby.node_index;
        best_cost = candidate_cost;
      }
    }

    // 即便使用最优父节点也不可能改善当前解，则整条分支可以丢弃。
    const double remaining_lower_bound = std::hypot(
      static_cast<double>(goal.x - new_node.x),
      static_cast<double>(goal.y - new_node.y));
    if (best_cost + remaining_lower_bound >= incumbent_cost) {continue;}

    inserted[new_grid_index] = true;
    const int new_index = static_cast<int>(nodes.size());
    nodes.push_back(new_node);
    parent.push_back(best_parent);
    cost_from_start.push_back(best_cost);
    children.emplace_back();
    children[static_cast<std::size_t>(best_parent)].push_back(new_index);
    result.visited.push_back(new_node);

    for (const auto & nearby : nearby_nodes) {
      const int neighbor_index = nearby.node_index;
      if (neighbor_index == best_parent || neighbor_index == 0) {continue;}
      const double rewired_cost = best_cost + nearby.distance;
      if (rewired_cost + 1e-9 >= cost_from_start[static_cast<std::size_t>(neighbor_index)]) {
        continue;
      }

      const int old_parent = parent[static_cast<std::size_t>(neighbor_index)];
      if (old_parent >= 0) {
        auto & old_children = children[static_cast<std::size_t>(old_parent)];
        old_children.erase(
          std::remove(old_children.begin(), old_children.end(), neighbor_index),
          old_children.end());
      }
      parent[static_cast<std::size_t>(neighbor_index)] = new_index;
      children[static_cast<std::size_t>(new_index)].push_back(neighbor_index);
      const double cost_change =
        rewired_cost - cost_from_start[static_cast<std::size_t>(neighbor_index)];
      std::vector<int> stack{neighbor_index};
      while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();
        cost_from_start[static_cast<std::size_t>(current)] += cost_change;
        for (const int child : children[static_cast<std::size_t>(current)]) {
          stack.push_back(child);
        }
      }
    }

    const double distance_to_goal = std::hypot(
      static_cast<double>(goal.x - new_node.x),
      static_cast<double>(goal.y - new_node.y));
    if (distance_to_goal <= step_size && lineTraversable(new_node, goal)) {
      const double solution_cost = best_cost + distance_to_goal;
      if (solution_cost < incumbent_cost) {
        incumbent_cost = solution_cost;
        incumbent_parent = new_index;
      }
    }
  }

  // 重连可能降低已有终点父节点的代价，结束时再完整检查一次确保取到最佳解。
  incumbent_cost = std::numeric_limits<double>::infinity();
  incumbent_parent = -1;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double distance_to_goal = std::hypot(
      static_cast<double>(goal.x - nodes[index].x),
      static_cast<double>(goal.y - nodes[index].y));
    if (distance_to_goal > step_size || !lineTraversable(nodes[index], goal)) {continue;}
    const double candidate_cost = cost_from_start[index] + distance_to_goal;
    if (candidate_cost < incumbent_cost) {
      incumbent_cost = candidate_cost;
      incumbent_parent = static_cast<int>(index);
    }
  }

  for (std::size_t index = 1; index < nodes.size(); ++index) {
    const int parent_index = parent[index];
    if (parent_index >= 0) {
      result.tree_edges.push_back({nodes[static_cast<std::size_t>(parent_index)], nodes[index]});
    }
  }
  if (incumbent_parent < 0) {return result;}

  result.tree_edges.push_back({nodes[static_cast<std::size_t>(incumbent_parent)], goal});
  result.path.push_back(goal);
  for (int current = incumbent_parent; current >= 0;
    current = parent[static_cast<std::size_t>(current)])
  {
    result.path.push_back(nodes[static_cast<std::size_t>(current)]);
    if (current == 0) {break;}
  }
  std::reverse(result.path.begin(), result.path.end());
  result.cost = incumbent_cost * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
