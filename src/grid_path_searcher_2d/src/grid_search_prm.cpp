/**
 * @file grid_search_prm.cpp
 * @brief 二维 PRM 概率路线图算法实现。
 *
 * 文件职责：随机采样自由格、连接无碰撞近邻，生成一张稀疏路线图，然后在
 * 路线图上运行 Dijkstra。与栅格 A* 不同，PRM 的搜索节点是随机采样点。
 *
 * 包含的函数：searchPrm()。
 */
#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <vector>

namespace grid_path_searcher_2d
{
namespace
{

struct RoadmapNeighbor
{
  int node_index;
  double distance;
};

struct RoadmapOpenNode
{
  double distance_from_start;
  int node_index;

  bool operator>(const RoadmapOpenNode & other) const
  {
    return distance_from_start > other.distance_from_start;
  }
};

}  // namespace

SearchResult GridSearcher2D::searchPrm(
  const GridIndex & start, const GridIndex & goal, const int sample_count,
  const int k_neighbors, const int seed) const
{
  SearchResult result;
  if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y) ||
    sample_count <= 0 || k_neighbors <= 0)
  {
    return result;
  }

  // 第一步：起点和终点必须是路线图节点。
  std::vector<GridIndex> nodes{start};
  if (start.x != goal.x || start.y != goal.y) {nodes.push_back(goal);}
  nodes.reserve(static_cast<std::size_t>(sample_count) + 2U);

  // 第二步：从自由空间随机采样，并用地图下标避免重复采到同一个格子。
  std::vector<bool> sampled(map_.data.size(), false);
  sampled[static_cast<std::size_t>(toLinear(start.x, start.y))] = true;
  sampled[static_cast<std::size_t>(toLinear(goal.x, goal.y))] = true;

  std::mt19937 random_generator(static_cast<std::mt19937::result_type>(seed));
  std::uniform_int_distribution<int> random_x(0, static_cast<int>(map_.info.width) - 1);
  std::uniform_int_distribution<int> random_y(0, static_cast<int>(map_.info.height) - 1);

  const int wanted_node_count = sample_count + (nodes.size() == 1U ? 1 : 2);
  const int maximum_attempts = sample_count * 60;
  for (int attempt = 0;
    attempt < maximum_attempts && static_cast<int>(nodes.size()) < wanted_node_count;
    ++attempt)
  {
    const GridIndex random_node{random_x(random_generator), random_y(random_generator)};
    const auto grid_index = static_cast<std::size_t>(toLinear(random_node.x, random_node.y));

    if (!traversable(random_node.x, random_node.y) || sampled[grid_index]) {continue;}
    sampled[grid_index] = true;
    nodes.push_back(random_node);
  }

  // graph[i] 保存路线图节点 i 能直接到达的邻居和边长。
  std::vector<std::vector<RoadmapNeighbor>> graph(nodes.size());
  std::vector<bool> edge_exists(nodes.size() * nodes.size(), false);

  // 第三步：每个采样点按距离寻找最近邻，只加入视线无碰撞的边。
  for (std::size_t from = 0; from < nodes.size(); ++from) {
    std::vector<RoadmapNeighbor> nearest_candidates;

    for (std::size_t to = 0; to < nodes.size(); ++to) {
      if (from == to) {continue;}
      const double dx = static_cast<double>(nodes[to].x - nodes[from].x);
      const double dy = static_cast<double>(nodes[to].y - nodes[from].y);
      nearest_candidates.push_back({static_cast<int>(to), std::hypot(dx, dy)});
    }

    std::sort(
      nearest_candidates.begin(), nearest_candidates.end(),
      [](const RoadmapNeighbor & left, const RoadmapNeighbor & right) {
        return left.distance < right.distance;
      });

    int connected_neighbors = 0;
    for (const auto & candidate : nearest_candidates) {
      if (connected_neighbors >= k_neighbors) {break;}

      const auto to = static_cast<std::size_t>(candidate.node_index);
      if (edge_exists[from * nodes.size() + to]) {
        ++connected_neighbors;
        continue;
      }
      if (!lineTraversable(nodes[from], nodes[to])) {continue;}

      graph[from].push_back(candidate);
      graph[to].push_back({static_cast<int>(from), candidate.distance});
      edge_exists[from * nodes.size() + to] = true;
      edge_exists[to * nodes.size() + from] = true;
      result.roadmap_edges.push_back({nodes[from], nodes[to]});
      ++connected_neighbors;
    }
  }

  // 第四步：在生成的路线图上运行标准 Dijkstra。
  const int goal_node_index = start.x == goal.x && start.y == goal.y ? 0 : 1;
  std::vector<double> distance(nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(nodes.size(), -1);
  std::vector<bool> closed(nodes.size(), false);
  std::priority_queue<
    RoadmapOpenNode, std::vector<RoadmapOpenNode>, std::greater<RoadmapOpenNode>> open_queue;

  distance[0] = 0.0;
  open_queue.push({0.0, 0});

  while (!open_queue.empty()) {
    const int current = open_queue.top().node_index;
    open_queue.pop();
    if (closed[static_cast<std::size_t>(current)]) {continue;}

    closed[static_cast<std::size_t>(current)] = true;
    result.visited.push_back(nodes[static_cast<std::size_t>(current)]);
    if (current == goal_node_index) {break;}

    for (const auto & neighbor : graph[static_cast<std::size_t>(current)]) {
      const double new_distance =
        distance[static_cast<std::size_t>(current)] + neighbor.distance;
      if (new_distance >= distance[static_cast<std::size_t>(neighbor.node_index)]) {continue;}

      distance[static_cast<std::size_t>(neighbor.node_index)] = new_distance;
      parent[static_cast<std::size_t>(neighbor.node_index)] = current;
      open_queue.push({new_distance, neighbor.node_index});
    }
  }

  if (!closed[static_cast<std::size_t>(goal_node_index)]) {return result;}

  // 第五步：从路线图终点沿 parent 回溯采样点路径。
  for (int current = goal_node_index; current >= 0;
    current = parent[static_cast<std::size_t>(current)])
  {
    result.path.push_back(nodes[static_cast<std::size_t>(current)]);
    if (current == 0) {break;}
  }
  std::reverse(result.path.begin(), result.path.end());

  result.cost = distance[static_cast<std::size_t>(goal_node_index)] * map_.info.resolution;
  result.success = true;
  return result;
}

}  // namespace grid_path_searcher_2d
