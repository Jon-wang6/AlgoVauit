/**
 * @file grid_search_informed_rrt_star.cpp
 * @brief 首解后在椭圆子集内采样的 Informed RRT* 实现。
 *
 * 首条路径出现前使用全地图均匀采样；出现首解后，以起点和终点为椭圆焦点，
 * 长轴由当前最优路径长度 c_best 决定、焦距由两点直线距离 c_min 决定，只在
 * 这个可能产生更优解的椭圆内采样，并继续执行 RRT* 择父与重连。
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

struct InformedNearbyNode
{
  int node_index;
  double distance;
};

}  // namespace

SearchResult GridSearcher2D::searchInformedRrtStar(
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
  std::uniform_real_distribution<double> uniform_zero_one(0.0, 1.0);
  std::uniform_real_distribution<double> random_angle(0.0, 2.0 * M_PI);
  const double safe_goal_bias = std::clamp(goal_bias, 0.0, 1.0);

  const double center_x = 0.5 * static_cast<double>(start.x + goal.x);
  const double center_y = 0.5 * static_cast<double>(start.y + goal.y);
  const double direction_x = static_cast<double>(goal.x - start.x);
  const double direction_y = static_cast<double>(goal.y - start.y);
  const double minimum_possible_cost = std::hypot(direction_x, direction_y);
  const double ellipse_rotation = std::atan2(direction_y, direction_x);
  double best_solution_cost = std::numeric_limits<double>::infinity();
  int best_goal_parent = -1;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    GridIndex random_sample;
    if (uniform_zero_one(generator) < safe_goal_bias) {
      random_sample = goal;
    } else if (std::isfinite(best_solution_cost)) {
      // 在单位圆内均匀采样，再缩放、旋转、平移到启发椭圆。
      const double radius = std::sqrt(uniform_zero_one(generator));
      const double angle = random_angle(generator);
      const double unit_x = radius * std::cos(angle);
      const double unit_y = radius * std::sin(angle);
      const double major_axis = best_solution_cost * 0.5;
      const double minor_axis =
        0.5 * std::sqrt(std::max(
        0.0, best_solution_cost * best_solution_cost -
        minimum_possible_cost * minimum_possible_cost));
      const double ellipse_x = major_axis * unit_x;
      const double ellipse_y = minor_axis * unit_y;
      random_sample.x = static_cast<int>(std::lround(
        center_x + std::cos(ellipse_rotation) * ellipse_x -
        std::sin(ellipse_rotation) * ellipse_y));
      random_sample.y = static_cast<int>(std::lround(
        center_y + std::sin(ellipse_rotation) * ellipse_x +
        std::cos(ellipse_rotation) * ellipse_y));
      if (!inBounds(random_sample.x, random_sample.y) ||
        !traversable(random_sample.x, random_sample.y))
      {
        continue;
      }
    } else {
      random_sample = {random_x(generator), random_y(generator)};
      if (!traversable(random_sample.x, random_sample.y)) {continue;}
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

    std::vector<InformedNearbyNode> nearby_nodes;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double distance = std::hypot(
        static_cast<double>(new_node.x - nodes[index].x),
        static_cast<double>(new_node.y - nodes[index].y));
      if (distance <= rewire_radius && lineTraversable(nodes[index], new_node)) {
        nearby_nodes.push_back({static_cast<int>(index), distance});
      }
    }

    int chosen_parent = nearest_index;
    double chosen_cost = cost_from_start[static_cast<std::size_t>(nearest_index)] +
      std::hypot(delta_x, delta_y);
    for (const auto & nearby : nearby_nodes) {
      const double candidate_cost =
        cost_from_start[static_cast<std::size_t>(nearby.node_index)] + nearby.distance;
      if (candidate_cost < chosen_cost) {
        chosen_parent = nearby.node_index;
        chosen_cost = candidate_cost;
      }
    }

    const double lower_bound_to_goal = std::hypot(
      static_cast<double>(goal.x - new_node.x),
      static_cast<double>(goal.y - new_node.y));
    if (chosen_cost + lower_bound_to_goal >= best_solution_cost) {continue;}

    inserted[new_grid_index] = true;
    const int new_index = static_cast<int>(nodes.size());
    nodes.push_back(new_node);
    parent.push_back(chosen_parent);
    cost_from_start.push_back(chosen_cost);
    children.emplace_back();
    children[static_cast<std::size_t>(chosen_parent)].push_back(new_index);
    result.visited.push_back(new_node);

    for (const auto & nearby : nearby_nodes) {
      const int neighbor_index = nearby.node_index;
      if (neighbor_index == chosen_parent || neighbor_index == 0) {continue;}
      const double rewired_cost = chosen_cost + nearby.distance;
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

    if (lower_bound_to_goal <= step_size && lineTraversable(new_node, goal)) {
      const double solution_cost = chosen_cost + lower_bound_to_goal;
      if (solution_cost < best_solution_cost) {
        best_solution_cost = solution_cost;
        best_goal_parent = new_index;
      }
    }
  }

  // 重连可能继续降低代价，结束时从所有可连接终点的节点中重新选最优者。
  best_solution_cost = std::numeric_limits<double>::infinity();
  best_goal_parent = -1;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double distance_to_goal = std::hypot(
      static_cast<double>(goal.x - nodes[index].x),
      static_cast<double>(goal.y - nodes[index].y));
    if (distance_to_goal > step_size || !lineTraversable(nodes[index], goal)) {continue;}
    const double candidate_cost = cost_from_start[index] + distance_to_goal;
    if (candidate_cost < best_solution_cost) {
      best_solution_cost = candidate_cost;
      best_goal_parent = static_cast<int>(index);
    }
  }

  for (std::size_t index = 1; index < nodes.size(); ++index) {
    const int parent_index = parent[index];
    if (parent_index >= 0) {
      result.tree_edges.push_back({nodes[static_cast<std::size_t>(parent_index)], nodes[index]});
    }
  }
  if (best_goal_parent < 0) {return result;}

  result.tree_edges.push_back({nodes[static_cast<std::size_t>(best_goal_parent)], goal});
  result.path.push_back(goal);
  for (int current = best_goal_parent; current >= 0;
    current = parent[static_cast<std::size_t>(current)])
  {
    result.path.push_back(nodes[static_cast<std::size_t>(current)]);
    if (current == 0) {break;}
  }
  std::reverse(result.path.begin(), result.path.end());
  result.cost = best_solution_cost * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
