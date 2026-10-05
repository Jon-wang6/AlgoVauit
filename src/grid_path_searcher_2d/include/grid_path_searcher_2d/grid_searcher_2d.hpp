/**
 * @file grid_searcher_2d.hpp
 * @brief 二维占据栅格搜索算法的数据结构与公开接口。
 *
 * 文件职责：定义二维栅格索引、搜索结果和 GridSearcher2D 类，供 ROS 2 节点
 * 导入 OccupancyGrid 后执行 A*、TimeBreak A*、Dijkstra、JPS、PRM、RRT 或 RRT*。
 *
 * 包含的类/结构：
 * - GridIndex：二维整数栅格坐标。
 * - GridEdge：PRM 路线图中的一条无碰撞连接边。
 * - SearchResult：搜索是否成功、路径、访问节点、JPS 跳点及采样算法边。
 * - GridSearcher2D：地图管理、坐标转换和十种路径搜索算法。
 *
 * 包含的函数：setMap()、worldToGrid()、gridToWorld()、searchAStar()、
 * searchTimedAStar()、searchDijkstra()、searchJps()、searchPrm()、searchRrt()、
 * searchRrtStar()，以及地图判断、路径回溯、跳点判断和视线检测辅助函数。
 * 各算法实现已按算法拆分到独立 cpp 文件，便于按笔记顺序单独阅读。
 */
#pragma once

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace grid_path_searcher_2d
{

/** 二维栅格坐标；x 向右递增，y 向上递增。 */
struct GridIndex
{
  int x{0};
  int y{0};
};

/** PRM 路线图中的无碰撞边；from 和 to 都是自由栅格采样点。 */
struct GridEdge
{
  GridIndex from;
  GridIndex to;
};

/** 一次搜索的完整输出，既供路径发布，也供 RViz 展示搜索过程。 */
struct SearchResult
{
  bool success{false};
  bool timed_out{false};
  double cost{0.0};
  std::vector<GridIndex> path;
  std::vector<GridIndex> visited;
  std::vector<GridIndex> jump_points;
  std::vector<GridEdge> roadmap_edges;
  std::vector<GridEdge> tree_edges;
};

/** 在 OccupancyGrid 上执行十种二维路径规划算法的搜索器。 */
class GridSearcher2D
{
public:
  /** 保存地图和通行参数；地图尺寸或 data 长度非法时返回 false。 */
  bool setMap(
    const nav_msgs::msg::OccupancyGrid & map, int occupied_threshold = 50,
    bool allow_unknown = false);

  /** 将世界坐标转换为地图栅格索引，并检查索引是否在地图内。 */
  bool worldToGrid(double world_x, double world_y, GridIndex & index) const;

  /** 返回指定栅格中心在世界坐标系中的坐标。 */
  geometry_msgs::msg::Point gridToWorld(const GridIndex & index) const;

  /** 运行 A*：按照 f = g + weight*h 选择节点；weight=1 为标准 A*。 */
  SearchResult searchAStar(
    const GridIndex & start, const GridIndex & goal,
    double heuristic_weight = 1.0) const;

  /**
   * 执行带 TimeBreak 的 A*；超过 time_limit_ms 后立即停止并返回 timed_out=true。
   * 时间预算只约束搜索计算，不包含 ROS 消息和 RViz 发布。
   */
  SearchResult searchTimedAStar(
    const GridIndex & start, const GridIndex & goal, double time_limit_ms,
    double heuristic_weight = 1.0) const;

  /** 运行 Dijkstra：按照起点累计代价 g 从小到大选择节点。 */
  SearchResult searchDijkstra(const GridIndex & start, const GridIndex & goal) const;

  /** 执行二维 Jump Point Search，并在结果中保留路径上的关键跳点。 */
  SearchResult searchJps(
    const GridIndex & start, const GridIndex & goal,
    double heuristic_weight = 1.0) const;

  /**
   * 执行概率路线图 PRM：采样自由格、连接 k 个近邻、碰撞检测并在路线图上
   * 运行 Dijkstra。seed 固定时，生成的路线图和路径可重复。
   */
  SearchResult searchPrm(
    const GridIndex & start, const GridIndex & goal, int sample_count,
    int k_neighbors, int seed) const;

  /**
   * 执行单树 RRT：在自由空间随机采样，从最近树节点按固定步长扩展，并以
   * goal_bias 概率直接采样终点；找到连接后通过父节点回溯路径。
   */
  SearchResult searchRrt(
    const GridIndex & start, const GridIndex & goal, int max_iterations,
    double step_size, double goal_bias, int seed) const;

  /**
   * 执行 RRT*：扩展新节点时在邻域内选择累计代价最低的父节点，然后对邻域
   * 节点执行重连以持续改进路径。rewire_radius 以栅格为单位。
   */
  SearchResult searchRrtStar(
    const GridIndex & start, const GridIndex & goal, int max_iterations,
    double step_size, double rewire_radius, double goal_bias, int seed) const;

  /**
   * 执行 Kinodynamic-RRT*：节点状态包含位置与速度，控制输入是有上限的二维
   * 加速度；按双积分模型推进状态，并只接受无碰撞且动力学可达的择父/重连边。
   */
  SearchResult searchKinodynamicRrtStar(
    const GridIndex & start, const GridIndex & goal, int max_iterations,
    double time_step, double max_speed, double max_acceleration,
    double rewire_radius, double goal_bias, int seed) const;

  /**
   * 执行 Anytime-RRT*：得到首条路径后继续利用剩余迭代和时间预算改进，并用
   * 当前最优代价做分支限界剪枝；返回截止时刻保存的最好路径。
   */
  SearchResult searchAnytimeRrtStar(
    const GridIndex & start, const GridIndex & goal, int max_iterations,
    double time_budget_ms, double step_size, double rewire_radius,
    double goal_bias, int seed) const;

  /**
   * 执行 Informed RRT*：首解之前全局采样，首解之后只在以起终点为焦点、由
   * 当前最优路径代价限定的椭圆内采样，从而集中计算资源继续优化路径。
   */
  SearchResult searchInformedRrtStar(
    const GridIndex & start, const GridIndex & goal, int max_iterations,
    double step_size, double rewire_radius, double goal_bias, int seed) const;

private:
  /** A* 和 Dijkstra 的队列排序规则；显式枚举比 true/false 更容易阅读。 */
  enum class PriorityMode
  {
    kDijkstra,
    kAStar
  };

  /** A* 与 Dijkstra 共用的八邻域松弛循环，由 PriorityMode 决定队列优先级。 */
  SearchResult searchWithPriorityQueue(
    const GridIndex & start, const GridIndex & goal, PriorityMode mode,
    double heuristic_weight) const;

  /** 判断索引是否位于地图边界内。 */
  bool inBounds(int x, int y) const;

  /** 将二维索引转换为 OccupancyGrid.data 的一维下标。 */
  int toLinear(int x, int y) const;

  /** 按占据阈值和未知区域策略判断单元是否可通行。 */
  bool traversable(int x, int y) const;

  /** 使用 octile distance 估计八邻域到目标的最小代价。 */
  static double heuristic(const GridIndex & from, const GridIndex & to);

  /** 按父节点表从终点回溯到起点，并反转为“起点到终点”的正向路径。 */
  std::vector<GridIndex> reconstructPath(
    const std::vector<int> & parent, int start_linear, int goal_linear) const;

  /** 判断当前跳跃方向在指定单元是否产生强迫邻居。 */
  bool hasForcedNeighbor(const GridIndex & index, int dx, int dy) const;

  /**
   * 从当前节点沿方向持续前进，直到碰到障碍、目标或强迫邻居；找到跳点时
   * 写入 jump_point 并返回 true。
   */
  bool jump(
    const GridIndex & current, int dx, int dy, const GridIndex & goal,
    GridIndex & jump_point) const;

  /** 使用 Bresenham 栅格遍历检查两采样点之间的直线是否完全无碰撞。 */
  bool lineTraversable(const GridIndex & from, const GridIndex & to) const;

  nav_msgs::msg::OccupancyGrid map_;
  int occupied_threshold_{50};
  bool allow_unknown_{false};
  bool ready_{false};
};

}  // namespace grid_path_searcher_2d
