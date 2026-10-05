/**
 * @file random_map_node.cpp
 * @brief 二维随机 OccupancyGrid 地图生成与发布节点。
 *
 * 文件职责：生成边界墙、随机矩形障碍和随机圆形障碍，清理起点/终点及一条
 * 保底通道，然后以 Transient Local QoS 周期发布地图。map.seed 小于 0 时每次
 * 启动自动选取新种子；非负时使用固定种子复现实验。
 *
 * 包含的类：RandomMapNode，负责地图参数、障碍绘制、安全区域和消息发布。
 * 包含的函数：RandomMapNode()、inBounds()、toLinear()、setCell()、fillCircle()、
 * fillRectangle()、clearLine()、generateMap()、publishMap() 和 main()。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class RandomMapNode : public rclcpp::Node
{
public:
  /** 读取地图参数、创建持久化发布器、生成地图并启动周期发布定时器。 */
  RandomMapNode() : Node("random_map_2d")
  {
    resolution_ = declare_parameter("map.resolution", 0.2);
    width_m_ = declare_parameter("map.width", 20.0);
    height_m_ = declare_parameter("map.height", 20.0);
    obstacle_count_ = declare_parameter("map.obstacle_count", 55);
    seed_ = declare_parameter("map.seed", -1);
    start_x_ = declare_parameter("planning.start_x", -8.0);
    start_y_ = declare_parameter("planning.start_y", -8.0);
    goal_x_ = declare_parameter("planning.goal_x", 8.0);
    goal_y_ = declare_parameter("planning.goal_y", 8.0);

    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/global_map", rclcpp::QoS(1).reliable().transient_local());
    generateMap();
    timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {publishMap();});
    publishMap();
    RCLCPP_INFO(
      get_logger(), "2D random map ready: %u x %u cells, resolution %.2f m, seed=%u",
      map_.info.width, map_.info.height, map_.info.resolution, effective_seed_);
  }

private:
  /** 检查整数栅格坐标是否位于地图范围内。 */
  bool inBounds(const int x, const int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
  }

  /** 把二维索引转换成地图 data 数组下标。 */
  int toLinear(const int x, const int y) const
  {
    return y * static_cast<int>(map_.info.width) + x;
  }

  /** 若坐标有效，则把指定单元写成占据值或空闲值。 */
  void setCell(const int x, const int y, const int8_t value)
  {
    if (inBounds(x, y)) {map_.data[static_cast<std::size_t>(toLinear(x, y))] = value;}
  }

  /** 在栅格坐标中填充圆形区域。 */
  void fillCircle(const int center_x, const int center_y, const int radius, const int8_t value)
  {
    for (int y = center_y - radius; y <= center_y + radius; ++y) {
      for (int x = center_x - radius; x <= center_x + radius; ++x) {
        const int dx = x - center_x;
        const int dy = y - center_y;
        if (dx * dx + dy * dy <= radius * radius) {setCell(x, y, value);}
      }
    }
  }

  /** 在栅格坐标中填充轴对齐矩形区域。 */
  void fillRectangle(
    const int center_x, const int center_y, const int half_width, const int half_height,
    const int8_t value)
  {
    for (int y = center_y - half_height; y <= center_y + half_height; ++y) {
      for (int x = center_x - half_width; x <= center_x + half_width; ++x) {
        setCell(x, y, value);
      }
    }
  }

  /** 沿线段采样并清理给定半径，构造一定可通行的弯折通道。 */
  void clearLine(
    const int x0, const int y0, const int x1, const int y1, const int radius)
  {
    const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    for (int step = 0; step <= steps; ++step) {
      const double ratio = steps == 0 ? 0.0 : static_cast<double>(step) / steps;
      const int x = static_cast<int>(std::lround(x0 + ratio * (x1 - x0)));
      const int y = static_cast<int>(std::lround(y0 + ratio * (y1 - y0)));
      fillCircle(x, y, radius, 0);
    }
  }

  /**
   * 地图生成步骤：初始化空地图和边界墙；seed<0 时将 random_device 与当前时间
   * 混合成新种子，否则使用指定种子；随后绘制两类障碍并清理保底通道。
   */
  void generateMap()
  {
    map_.header.frame_id = "map";
    map_.info.resolution = static_cast<float>(resolution_);
    map_.info.width = static_cast<uint32_t>(std::lround(width_m_ / resolution_));
    map_.info.height = static_cast<uint32_t>(std::lround(height_m_ / resolution_));
    map_.info.origin.position.x = -width_m_ / 2.0;
    map_.info.origin.position.y = -height_m_ / 2.0;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign(static_cast<std::size_t>(map_.info.width) * map_.info.height, 0);

    for (int x = 0; x < static_cast<int>(map_.info.width); ++x) {
      setCell(x, 0, 100);
      setCell(x, static_cast<int>(map_.info.height) - 1, 100);
    }
    for (int y = 0; y < static_cast<int>(map_.info.height); ++y) {
      setCell(0, y, 100);
      setCell(static_cast<int>(map_.info.width) - 1, y, 100);
    }

    if (seed_ < 0) {
      std::random_device random_device;
      const auto clock_value = static_cast<std::uint64_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
      effective_seed_ = static_cast<std::uint32_t>(
        random_device() ^ static_cast<std::uint32_t>(clock_value) ^
        static_cast<std::uint32_t>(clock_value >> 32U));
    } else {
      effective_seed_ = static_cast<std::uint32_t>(seed_);
    }
    std::mt19937 generator(effective_seed_);
    std::uniform_int_distribution<int> x_distribution(5, static_cast<int>(map_.info.width) - 6);
    std::uniform_int_distribution<int> y_distribution(5, static_cast<int>(map_.info.height) - 6);
    std::uniform_int_distribution<int> size_distribution(2, 6);
    for (int obstacle = 0; obstacle < obstacle_count_; ++obstacle) {
      const int x = x_distribution(generator);
      const int y = y_distribution(generator);
      const int size = size_distribution(generator);
      if (obstacle % 3 == 0) {
        fillCircle(x, y, size, 100);
      } else {
        fillRectangle(x, y, size, std::max(1, size / 2), 100);
      }
    }

    const auto to_x = [this](const double world_x) {
        return static_cast<int>(std::floor((world_x - map_.info.origin.position.x) / resolution_));
      };
    const auto to_y = [this](const double world_y) {
        return static_cast<int>(std::floor((world_y - map_.info.origin.position.y) / resolution_));
      };
    const std::vector<std::pair<double, double>> corridor{
      {start_x_, start_y_}, {-5.5, -3.0}, {-2.0, -5.0},
      {1.0, -0.5}, {4.0, 2.0}, {3.0, 6.0}, {goal_x_, goal_y_}};
    const int corridor_radius = std::max(2, static_cast<int>(std::ceil(0.45 / resolution_)));
    for (std::size_t i = 1; i < corridor.size(); ++i) {
      clearLine(
        to_x(corridor[i - 1].first), to_y(corridor[i - 1].second),
        to_x(corridor[i].first), to_y(corridor[i].second), corridor_radius);
    }
    fillCircle(to_x(start_x_), to_y(start_y_), corridor_radius + 2, 0);
    fillCircle(to_x(goal_x_), to_y(goal_y_), corridor_radius + 2, 0);
  }

  /** 更新时间戳后发布完整 OccupancyGrid。 */
  void publishMap()
  {
    map_.header.stamp = now();
    publisher_->publish(map_);
  }

  double resolution_{0.2};
  double width_m_{20.0};
  double height_m_{20.0};
  double start_x_{-8.0};
  double start_y_{-8.0};
  double goal_x_{8.0};
  double goal_y_{8.0};
  int obstacle_count_{55};
  int seed_{-1};
  std::uint32_t effective_seed_{0};
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/** 创建 RandomMapNode 并交给 ROS 2 执行器持续运行。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomMapNode>());
  rclcpp::shutdown();
  return 0;
}
