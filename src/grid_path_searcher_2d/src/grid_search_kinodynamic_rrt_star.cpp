/**
 * @file grid_search_kinodynamic_rrt_star.cpp
 * @brief 二维双积分器模型的 Kinodynamic-RRT* 教学实现。
 *
 * 状态为 (x, y, vx, vy)，控制量为 (ax, ay)。每次扩展使用
 * p'=p+v*dt+0.5*a*dt^2、v'=v+a*dt 推进，并限制速度和加速度；碰撞检测覆盖
 * 整段运动。算法随后在动力学可达的邻域内择父与重连，而不是把任意两点直连。
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

/** 树节点保存连续位置、速度、父节点和从起点累计的运动代价。 */
struct DynamicNode
{
  double x{0.0};
  double y{0.0};
  double velocity_x{0.0};
  double velocity_y{0.0};
  int parent{-1};
  double cost{0.0};
  std::vector<int> children;
};

/** 一次加速度控制推进的结果。 */
struct Propagation
{
  DynamicNode state;
  double acceleration_norm{0.0};
};

/** 将二维向量的模限制在 maximum 内。 */
void limitVector(double & x, double & y, const double maximum)
{
  const double norm = std::hypot(x, y);
  if (norm <= maximum || norm < 1e-12) {return;}
  x *= maximum / norm;
  y *= maximum / norm;
}

/** 按双积分器方程，从 from 朝目标位置推进一个时间步。 */
Propagation propagateToward(
  const DynamicNode & from, const double target_x, const double target_y,
  const double time_step, const double max_speed, const double max_acceleration)
{
  double acceleration_x =
    2.0 * (target_x - from.x - from.velocity_x * time_step) /
    (time_step * time_step);
  double acceleration_y =
    2.0 * (target_y - from.y - from.velocity_y * time_step) /
    (time_step * time_step);
  limitVector(acceleration_x, acceleration_y, max_acceleration);

  Propagation propagation;
  propagation.acceleration_norm = std::hypot(acceleration_x, acceleration_y);
  propagation.state.x =
    from.x + from.velocity_x * time_step +
    0.5 * acceleration_x * time_step * time_step;
  propagation.state.y =
    from.y + from.velocity_y * time_step +
    0.5 * acceleration_y * time_step * time_step;
  propagation.state.velocity_x = from.velocity_x + acceleration_x * time_step;
  propagation.state.velocity_y = from.velocity_y + acceleration_y * time_step;
  limitVector(
    propagation.state.velocity_x, propagation.state.velocity_y, max_speed);
  return propagation;
}

GridIndex roundedCell(const DynamicNode & node)
{
  return {
    static_cast<int>(std::lround(node.x)),
    static_cast<int>(std::lround(node.y))};
}

}  // namespace

SearchResult GridSearcher2D::searchKinodynamicRrtStar(
  const GridIndex & start, const GridIndex & goal, const int max_iterations,
  const double time_step, const double max_speed, const double max_acceleration,
  const double rewire_radius, const double goal_bias, const int seed) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    max_iterations <= 0 || time_step <= 0.0 || max_speed <= 0.0 ||
    max_acceleration <= 0.0 || rewire_radius <= 0.0)
  {
    return result;
  }

  if (start.x == goal.x && start.y == goal.y) {
    result.success = true;
    result.path.push_back(start);
    return result;
  }

  std::vector<DynamicNode> nodes;
  nodes.push_back({
    static_cast<double>(start.x), static_cast<double>(start.y),
    0.0, 0.0, -1, 0.0, {}});
  result.visited.push_back(start);

  std::mt19937 generator(static_cast<std::mt19937::result_type>(seed));
  std::uniform_real_distribution<double> sample_x(
    0.0, static_cast<double>(map_.info.width - 1U));
  std::uniform_real_distribution<double> sample_y(
    0.0, static_cast<double>(map_.info.height - 1U));
  std::uniform_real_distribution<double> sample_velocity(-max_speed, max_speed);
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  const double safe_goal_bias = std::clamp(goal_bias, 0.0, 1.0);

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    // 一、在“位置+速度”状态空间采样；目标偏置时采样静止终点状态。
    double target_x = 0.0;
    double target_y = 0.0;
    double target_velocity_x = 0.0;
    double target_velocity_y = 0.0;
    if (probability(generator) < safe_goal_bias) {
      target_x = static_cast<double>(goal.x);
      target_y = static_cast<double>(goal.y);
    } else {
      target_x = sample_x(generator);
      target_y = sample_y(generator);
      target_velocity_x = sample_velocity(generator);
      target_velocity_y = sample_velocity(generator);
      const GridIndex sample_cell{
        static_cast<int>(std::lround(target_x)),
        static_cast<int>(std::lround(target_y))};
      if (!traversable(sample_cell.x, sample_cell.y)) {continue;}
    }

    // 二、Nearest：位置距离为主，速度差作为次要状态距离。
    int nearest_index = 0;
    double nearest_metric = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double dx = target_x - nodes[index].x;
      const double dy = target_y - nodes[index].y;
      const double dvx = target_velocity_x - nodes[index].velocity_x;
      const double dvy = target_velocity_y - nodes[index].velocity_y;
      const double metric = dx * dx + dy * dy + 0.15 * (dvx * dvx + dvy * dvy);
      if (metric < nearest_metric) {
        nearest_metric = metric;
        nearest_index = static_cast<int>(index);
      }
    }

    // 三、用受限加速度推进一个时间步，并检查整段轨迹是否碰撞。
    Propagation best_propagation = propagateToward(
      nodes[static_cast<std::size_t>(nearest_index)], target_x, target_y,
      time_step, max_speed, max_acceleration);
    GridIndex best_cell = roundedCell(best_propagation.state);
    const GridIndex nearest_cell = roundedCell(nodes[static_cast<std::size_t>(nearest_index)]);
    if (!traversable(best_cell.x, best_cell.y) ||
      (best_cell.x == nearest_cell.x && best_cell.y == nearest_cell.y) ||
      !lineTraversable(nearest_cell, best_cell))
    {
      continue;
    }

    // 四、Choose parent：邻域节点必须通过同样动力学推进后接近候选状态。
    int best_parent = nearest_index;
    const auto edgeCost = [time_step](
      const DynamicNode & from, const DynamicNode & to, const double acceleration) {
        return std::hypot(to.x - from.x, to.y - from.y) +
               0.02 * acceleration * time_step;
      };
    double best_cost = nodes[static_cast<std::size_t>(nearest_index)].cost + edgeCost(
      nodes[static_cast<std::size_t>(nearest_index)], best_propagation.state,
      best_propagation.acceleration_norm);

    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const double distance = std::hypot(
        nodes[index].x - best_propagation.state.x,
        nodes[index].y - best_propagation.state.y);
      if (distance > rewire_radius) {continue;}

      const auto candidate = propagateToward(
        nodes[index], best_propagation.state.x, best_propagation.state.y,
        time_step, max_speed, max_acceleration);
      const double position_error = std::hypot(
        candidate.state.x - best_propagation.state.x,
        candidate.state.y - best_propagation.state.y);
      const GridIndex from_cell = roundedCell(nodes[index]);
      const GridIndex candidate_cell = roundedCell(candidate.state);
      if (position_error > 1.0 || !traversable(candidate_cell.x, candidate_cell.y) ||
        !lineTraversable(from_cell, candidate_cell))
      {
        continue;
      }
      const double candidate_cost = nodes[index].cost +
        edgeCost(nodes[index], candidate.state, candidate.acceleration_norm);
      if (candidate_cost < best_cost) {
        best_cost = candidate_cost;
        best_parent = static_cast<int>(index);
        best_propagation = candidate;
        best_cell = candidate_cell;
      }
    }

    // 避免在几乎相同的位置和速度重复插入状态。
    bool duplicate = false;
    for (const auto & node : nodes) {
      if (std::hypot(node.x - best_propagation.state.x, node.y - best_propagation.state.y) < 0.35 &&
        std::hypot(
          node.velocity_x - best_propagation.state.velocity_x,
          node.velocity_y - best_propagation.state.velocity_y) < 0.35)
      {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {continue;}

    best_propagation.state.parent = best_parent;
    best_propagation.state.cost = best_cost;
    const int new_index = static_cast<int>(nodes.size());
    nodes.push_back(best_propagation.state);
    nodes[static_cast<std::size_t>(best_parent)].children.push_back(new_index);
    result.visited.push_back(best_cell);

    // 五、Rewire：新状态能在一个受限控制步内到达邻居且代价更低时才允许重连。
    for (int neighbor_index = 1; neighbor_index < new_index; ++neighbor_index) {
      auto & neighbor = nodes[static_cast<std::size_t>(neighbor_index)];
      if (std::hypot(neighbor.x - nodes.back().x, neighbor.y - nodes.back().y) > rewire_radius) {
        continue;
      }

      // 祖先不能改挂到自己的后代下面，否则会产生父节点环。
      bool neighbor_is_ancestor = false;
      for (int ancestor = best_parent; ancestor >= 0;
        ancestor = nodes[static_cast<std::size_t>(ancestor)].parent)
      {
        if (ancestor == neighbor_index) {neighbor_is_ancestor = true; break;}
      }
      if (neighbor_is_ancestor) {continue;}

      const auto rewired = propagateToward(
        nodes.back(), neighbor.x, neighbor.y, time_step, max_speed, max_acceleration);
      const double position_error = std::hypot(rewired.state.x - neighbor.x, rewired.state.y - neighbor.y);
      const double velocity_error = std::hypot(
        rewired.state.velocity_x - neighbor.velocity_x,
        rewired.state.velocity_y - neighbor.velocity_y);
      if (position_error > 0.9 || velocity_error > max_acceleration * time_step * 1.5 ||
        !lineTraversable(roundedCell(nodes.back()), roundedCell(rewired.state)))
      {
        continue;
      }

      const double rewired_cost = nodes.back().cost +
        edgeCost(nodes.back(), rewired.state, rewired.acceleration_norm);
      if (rewired_cost + 1e-9 >= neighbor.cost) {continue;}

      const int old_parent = neighbor.parent;
      if (old_parent >= 0) {
        auto & old_children = nodes[static_cast<std::size_t>(old_parent)].children;
        old_children.erase(
          std::remove(old_children.begin(), old_children.end(), neighbor_index),
          old_children.end());
      }
      neighbor.parent = new_index;
      nodes.back().children.push_back(neighbor_index);
      const double cost_change = rewired_cost - neighbor.cost;
      std::vector<int> stack{neighbor_index};
      while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();
        nodes[static_cast<std::size_t>(current)].cost += cost_change;
        for (const int child : nodes[static_cast<std::size_t>(current)].children) {
          stack.push_back(child);
        }
      }
    }
  }

  // 根据最终父节点关系生成动力学树。
  for (std::size_t index = 1; index < nodes.size(); ++index) {
    const int parent = nodes[index].parent;
    if (parent >= 0) {
      result.tree_edges.push_back({roundedCell(nodes[static_cast<std::size_t>(parent)]), roundedCell(nodes[index])});
    }
  }

  // 终点是一个区域：进入该区域且最后一段无碰撞即可完成规划。
  const double goal_tolerance = std::max(2.0, max_speed * time_step);
  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double distance = std::hypot(
      nodes[index].x - static_cast<double>(goal.x),
      nodes[index].y - static_cast<double>(goal.y));
    if (distance > goal_tolerance || !lineTraversable(roundedCell(nodes[index]), goal)) {
      continue;
    }
    const double candidate_cost = nodes[index].cost + distance;
    if (candidate_cost < best_goal_cost) {
      best_goal_cost = candidate_cost;
      best_goal_parent = static_cast<int>(index);
    }
  }
  if (best_goal_parent < 0) {return result;}

  result.tree_edges.push_back({roundedCell(nodes[static_cast<std::size_t>(best_goal_parent)]), goal});
  result.path.push_back(goal);
  for (int current = best_goal_parent; current >= 0;
    current = nodes[static_cast<std::size_t>(current)].parent)
  {
    const GridIndex cell = roundedCell(nodes[static_cast<std::size_t>(current)]);
    if (result.path.empty() || result.path.back().x != cell.x || result.path.back().y != cell.y) {
      result.path.push_back(cell);
    }
    if (current == 0) {break;}
  }
  std::reverse(result.path.begin(), result.path.end());
  result.cost = best_goal_cost * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
