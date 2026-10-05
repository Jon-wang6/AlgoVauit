/**
 * @file occupancy_demo_node.cpp
 * @brief ROS 1 教程二维 OccupancyGrid 数据的 ROS 2 兼容规划节点。
 *
 * 文件职责：订阅二维占据地图、里程计和航点，在八邻域栅格上运行 A*，
 * 发布 nav_msgs/Path、路径点 Marker 及供 RViz2 显示的地图。
 *
 * 包含的类/结构：
 * - Cell：二维栅格索引。
 * - QueueEntry：A* 开放队列元素及最小代价比较规则。
 * - GridPathSearcher：二维地图接入、A* 搜索和结果发布节点。
 *
 * 包含的函数：
 * - GridPathSearcher()：读取参数并建立发布/订阅关系。
 * - mapCallback()/waypointCallback()/odomCallback()：更新地图、目标和起点。
 * - worldToCell()/cellToWorld()：世界坐标与栅格坐标互换。
 * - inBounds()/index()/traversable()：完成边界、线性索引和通行性检查。
 * - search()：执行八邻域 A* 并回溯栅格路径。
 * - planAndPublish()：校验输入、调用搜索并发布 ROS 消息。
 * - main()：初始化 ROS 2、运行节点并完成退出清理。
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{
// 二维栅格坐标和 A* 优先队列元素，只在当前翻译单元内使用。
struct Cell {int x; int y;};
struct QueueEntry
{
  double f;
  int index;
  // priority_queue 默认取最大值，这里反转比较以得到最小 fScore。
  bool operator>(const QueueEntry & other) const {return f > other.f;}
};
}  // namespace

// 二维兼容规划节点：在 nav_msgs/OccupancyGrid 上执行八邻域 A*。
class GridPathSearcher : public rclcpp::Node
{
public:
  /** 读取地图/里程计参数，建立三个订阅器和地图、路径、Marker 发布器。 */
  GridPathSearcher() : Node("demo_node")
  {
    const auto map_topic = declare_parameter<std::string>("map_topic", "/grid_map_global");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/laser_localization");
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    allow_unknown_ = declare_parameter<bool>("allow_unknown", true);
    replan_on_map_update_ = declare_parameter<bool>("replan_on_map_update", true);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, map_qos, std::bind(&GridPathSearcher::mapCallback, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      "~/waypoints", 10,
      std::bind(&GridPathSearcher::waypointCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, std::bind(&GridPathSearcher::odomCallback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/grid_path_vis", 10);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/grid_map", rclcpp::QoS(1).reliable().transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    RCLCPP_INFO(get_logger(), "Grid planner ready: map=%s, odom=%s", map_topic.c_str(), odom_topic.c_str());
  }

private:
  /** 保存最新二维地图、转发给 RViz，并按配置决定是否立即重规划。 */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;
    have_map_ = true;
    map_pub_->publish(map_);
    if (replan_on_map_update_) {planAndPublish();}
  }

  /** 取航点路径的最后一个姿态作为目标，并触发一次规划。 */
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty waypoint path");
      return;
    }
    goal_ = msg->poses.back();
    have_goal_ = true;
    path_published_for_goal_ = false;
    planAndPublish();
  }

  /** 将里程计姿态保存为当前起点；目标尚未生成路径时尝试规划。 */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    have_odom_ = true;
    if (!path_published_for_goal_) {planAndPublish();}
  }

  /**
   * 世界坐标转栅格坐标：先减地图原点，再逆旋转到地图局部坐标，
   * 最后除以分辨率并检查是否越界。
   */
  bool worldToCell(double wx, double wy, Cell & cell) const
  {
    const auto & origin = map_.info.origin;
    tf2::Quaternion quaternion;
    tf2::fromMsg(origin.orientation, quaternion);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    const double dx = wx - origin.position.x;
    const double dy = wy - origin.position.y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (map_.info.resolution <= 0.0) {return false;}
    cell.x = static_cast<int>(std::floor(local_x / map_.info.resolution));
    cell.y = static_cast<int>(std::floor(local_y / map_.info.resolution));
    return inBounds(cell.x, cell.y);
  }

  /** 栅格坐标转世界坐标：取单元中心，再应用地图原点的旋转和平移。 */
  geometry_msgs::msg::Point cellToWorld(const Cell & cell) const
  {
    const auto & origin = map_.info.origin;
    tf2::Quaternion quaternion;
    tf2::fromMsg(origin.orientation, quaternion);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    const double local_x = (static_cast<double>(cell.x) + 0.5) * map_.info.resolution;
    const double local_y = (static_cast<double>(cell.y) + 0.5) * map_.info.resolution;
    geometry_msgs::msg::Point point;
    point.x = origin.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y;
    point.y = origin.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y;
    point.z = origin.position.z;
    return point;
  }

  /** 判断二维索引是否位于地图宽高范围内。 */
  bool inBounds(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
  }

  /** 将二维栅格索引压平成 OccupancyGrid.data 使用的一维索引。 */
  int index(int x, int y) const {return y * static_cast<int>(map_.info.width) + x;}

  /** 根据占据阈值和 allow_unknown 参数判断单元是否可以通行。 */
  bool traversable(int x, int y) const
  {
    const int8_t value = map_.data[static_cast<std::size_t>(index(x, y))];
    return value < 0 ? allow_unknown_ : value < occupied_threshold_;
  }

  /**
   * 执行八邻域 A*：
   * 1. 创建 gScore、父节点、关闭集和最小优先队列；
   * 2. 每次取 f=g+h 最小的单元，检查 8 个相邻单元；
   * 3. 松弛更短路径并以欧氏距离作为启发项；
   * 4. 到达目标后沿 parent 反向回溯，再翻转成起点到终点顺序。
   */
  std::vector<Cell> search(const Cell & start, const Cell & goal) const
  {
    if (!traversable(start.x, start.y) || !traversable(goal.x, goal.y)) {return {};}
    const std::size_t count = map_.data.size();
    std::vector<double> g_score(count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(count, -1);
    std::vector<bool> closed(count, false);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    const int start_index = index(start.x, start.y);
    const int goal_index = index(goal.x, goal.y);
    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    open.push({0.0, start_index});
    constexpr int directions[8][3] = {
      {1, 0, 10}, {-1, 0, 10}, {0, 1, 10}, {0, -1, 10},
      {1, 1, 14}, {1, -1, 14}, {-1, 1, 14}, {-1, -1, 14}};

    while (!open.empty()) {
      const int current = open.top().index;
      open.pop();
      if (closed[static_cast<std::size_t>(current)]) {continue;}
      closed[static_cast<std::size_t>(current)] = true;
      if (current == goal_index) {break;}
      const int x = current % static_cast<int>(map_.info.width);
      const int y = current / static_cast<int>(map_.info.width);
      for (const auto & direction : directions) {
        const int nx = x + direction[0];
        const int ny = y + direction[1];
        if (!inBounds(nx, ny) || !traversable(nx, ny)) {continue;}
        const int neighbor = index(nx, ny);
        const double candidate = g_score[static_cast<std::size_t>(current)] + direction[2];
        if (candidate >= g_score[static_cast<std::size_t>(neighbor)]) {continue;}
        parent[static_cast<std::size_t>(neighbor)] = current;
        g_score[static_cast<std::size_t>(neighbor)] = candidate;
        const double dx = static_cast<double>(goal.x - nx);
        const double dy = static_cast<double>(goal.y - ny);
        open.push({candidate + 10.0 * std::hypot(dx, dy), neighbor});
      }
    }
    if (start_index != goal_index && parent[static_cast<std::size_t>(goal_index)] < 0) {return {};}
    std::vector<Cell> result;
    for (int current = goal_index; current >= 0; current = parent[static_cast<std::size_t>(current)]) {
      result.push_back({current % static_cast<int>(map_.info.width), current / static_cast<int>(map_.info.width)});
      if (current == start_index) {break;}
    }
    std::reverse(result.begin(), result.end());
    return result;
  }

  /**
   * 当地图、里程计和目标均就绪时组织一次规划与发布：
   * 校验地图数据 → 坐标转换 → A* → 转换为 nav_msgs/Path 和 Marker → 发布。
   */
  void planAndPublish()
  {
    if (!have_map_ || !have_odom_ || !have_goal_) {return;}
    if (map_.data.size() != static_cast<std::size_t>(map_.info.width) * map_.info.height) {
      RCLCPP_ERROR(get_logger(), "OccupancyGrid dimensions do not match data length");
      return;
    }
    Cell start{}, goal{};
    if (!worldToCell(current_pose_.pose.position.x, current_pose_.pose.position.y, start)) {
      RCLCPP_WARN(get_logger(), "Current odometry pose is outside the map");
      return;
    }
    if (!worldToCell(goal_.pose.position.x, goal_.pose.position.y, goal)) {
      RCLCPP_WARN(get_logger(), "Goal pose is outside the map");
      return;
    }
    const auto cells = search(start, goal);
    if (cells.empty()) {
      RCLCPP_WARN(get_logger(), "No path from (%d,%d) to (%d,%d)", start.x, start.y, goal.x, goal.y);
      return;
    }
    const auto stamp = now();
    const std::string frame = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = frame;
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "grid_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_.info.resolution * 0.7;
    marker.scale.y = map_.info.resolution * 0.7;
    marker.scale.z = std::max(0.05, static_cast<double>(map_.info.resolution) * 0.25);
    marker.color.a = 1.0F;
    marker.color.r = 0.5F;
    marker.color.b = 0.8F;
    for (const auto & cell : cells) {
      const auto point = cellToWorld(cell);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
      marker.points.push_back(point);
    }
    path_pub_->publish(path);
    marker_pub_->publish(marker);
    path_published_for_goal_ = true;
    RCLCPP_INFO(get_logger(), "Published path with %zu poses", path.poses.size());
  }

  int occupied_threshold_{50};
  bool allow_unknown_{true}, replan_on_map_update_{true};
  bool have_map_{false}, have_odom_{false}, have_goal_{false}, path_published_for_goal_{false};
  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped current_pose_, goal_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

/** 二维兼容节点入口。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridPathSearcher>());
  rclcpp::shutdown();
  return 0;
}
