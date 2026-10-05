/**
 * @file random_complex_node.cpp
 * @brief 可复现的三维随机障碍地图生成节点。
 *
 * 文件职责：根据地图尺寸、障碍数量和随机种子生成柱体与空间圆环，避开
 * 起点/默认终点的安全区，并以 PointCloud2 周期发布给路径规划节点。
 *
 * 包含的类：
 * - RandomComplexNode：负责采样、缓存和发布三维障碍点云的 ROS 2 节点。
 *
 * 包含的函数：
 * - RandomComplexNode()：读取参数，生成地图并创建发布器与定时器。
 * - clearAround()：检查候选障碍是否避开起点和默认终点。
 * - addPoint()：过滤越界点并加入地图点集。
 * - generateMap()：按随机种子生成柱体和圆环障碍。
 * - publishMap()：将缓存点集编码成 PointCloud2 并发布。
 * - main()：初始化 ROS 2、运行节点并完成退出清理。
 */
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using Eigen::Matrix3d;
using Eigen::Vector3d;

// 三维随机地图节点：生成柱体和空间圆环，并周期发布为 PointCloud2。
class RandomComplexNode : public rclcpp::Node
{
public:
  /**
   * 初始化地图参数、发布器和定时器。
   * 固定 seed 可让每次启动得到相同地图，便于算法对比和复现实验。
   */
  RandomComplexNode() : Node("random_complex")
  {
    x_size_ = declare_parameter("map.x_size", 20.0);
    y_size_ = declare_parameter("map.y_size", 20.0);
    z_size_ = declare_parameter("map.z_size", 4.0);
    resolution_ = declare_parameter("map.resolution", 0.2);
    const int obstacle_count = declare_parameter("map.obstacle_count", 180);
    const int ring_count = declare_parameter("map.ring_count", 32);
    const int seed = declare_parameter("map.seed", 7);
    start_x_ = declare_parameter("init_state_x", 0.0);
    start_y_ = declare_parameter("init_state_y", 0.0);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/global_map", rclcpp::QoS(1).reliable().transient_local());
    generateMap(obstacle_count, ring_count, seed);
    timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {publishMap();});
    publishMap();
    RCLCPP_INFO(get_logger(), "random_complex ready: %zu point-cloud obstacles", points_.size());
  }

private:
  /** 判断候选障碍是否远离起点和默认目标，避免一启动就把两端堵死。 */
  bool clearAround(const double x, const double y) const
  {
    const auto clear = [x, y](const double cx, const double cy) {
        return std::hypot(x - cx, y - cy) > 1.25;
      };
    return clear(start_x_, start_y_) && clear(7.0, 7.0);
  }

  /** 仅将地图边界内的点加入点云，边界外采样直接丢弃。 */
  void addPoint(const Vector3d & point)
  {
    if (point.x() >= -x_size_ / 2.0 && point.x() < x_size_ / 2.0 &&
      point.y() >= -y_size_ / 2.0 && point.y() < y_size_ / 2.0 &&
      point.z() >= 0.0 && point.z() < z_size_)
    {
      points_.push_back(point);
    }
  }

  /**
   * 生成随机复杂地图：
   * 1. 用 seed 初始化随机数引擎及位置、尺寸、高度分布；
   * 2. 采样椭圆环，对环做随机三维旋转后加入点云；
   * 3. 采样柱体中心，将宽度和高度离散到 resolution 栅格；
   * 4. 过滤起点、默认目标附近和地图范围外的点。
   */
  void generateMap(const int obstacle_count, const int ring_count, const int seed)
  {
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> random_x(-x_size_ / 2.0, x_size_ / 2.0);
    std::uniform_real_distribution<double> random_y(-y_size_ / 2.0, y_size_ / 2.0);
    std::uniform_real_distribution<double> random_width(0.12, 0.65);
    std::uniform_real_distribution<double> random_height(1.0, z_size_);
    std::uniform_real_distribution<double> random_radius(0.7, 1.8);
    std::uniform_real_distribution<double> random_angle(-M_PI, M_PI);
    std::uniform_real_distribution<double> random_tilt(M_PI / 4.0, M_PI / 2.0);

    for (int i = 0; i < ring_count; ++i) {
      const double cx = random_x(random);
      const double cy = random_y(random);
      if (!clearAround(cx, cy)) {continue;}
      const double radius = random_radius(random);
      const double a = 0.6 + random_radius(random) * 0.6;
      const double b = 0.6 + random_radius(random) * 0.6;
      const double alpha = random_angle(random);
      const double beta = random_tilt(random);
      const double gamma = random_tilt(random);
      Matrix3d rotation;
      rotation <<
        std::cos(alpha) * std::cos(gamma) - std::cos(beta) * std::sin(alpha) * std::sin(gamma),
        -std::cos(beta) * std::cos(gamma) * std::sin(alpha) - std::cos(alpha) * std::sin(gamma),
        std::sin(alpha) * std::sin(beta),
        std::cos(gamma) * std::sin(alpha) + std::cos(alpha) * std::cos(beta) * std::sin(gamma),
        std::cos(alpha) * std::cos(beta) * std::cos(gamma) - std::sin(alpha) * std::sin(gamma),
        -std::cos(alpha) * std::sin(beta),
        std::sin(beta) * std::sin(gamma), std::cos(gamma) * std::sin(beta), std::cos(beta);
      const double center_z = random_height(random) * 0.45;
      for (double theta = -M_PI; theta < M_PI; theta += 0.035) {
        Vector3d point = rotation * Vector3d(
          a * radius * std::cos(theta), b * radius * std::sin(theta), 0.0);
        point += Vector3d(cx, cy, center_z);
        if (clearAround(point.x(), point.y())) {addPoint(point);}
      }
    }

    for (int i = 0; i < obstacle_count; ++i) {
      double x = random_x(random);
      double y = random_y(random);
      if (!clearAround(x, y)) {continue;}
      x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
      y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;
      const int width_cells = std::max(
        1, static_cast<int>(std::ceil(random_width(random) / resolution_)));
      for (int ix = -width_cells / 2; ix <= width_cells / 2; ++ix) {
        for (int iy = -width_cells / 2; iy <= width_cells / 2; ++iy) {
          const int height_cells = static_cast<int>(
            std::ceil(random_height(random) / resolution_));
          for (int iz = 0; iz < height_cells; ++iz) {
            addPoint(Vector3d(
              x + ix * resolution_, y + iy * resolution_, (iz + 0.5) * resolution_));
          }
        }
      }
    }
  }

  /**
   * 把 Eigen 点集合封装成 sensor_msgs/PointCloud2。
   * 设置 world 坐标系与时间戳，声明 xyz 字段，逐点写入后发布。
   */
  void publishMap()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points_.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points_) {
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
      ++x;
      ++y;
      ++z;
    }
    publisher_->publish(cloud);
  }

  double x_size_{};
  double y_size_{};
  double z_size_{};
  double resolution_{};
  double start_x_{};
  double start_y_{};
  std::vector<Vector3d> points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/** ROS 2 程序入口：创建 random_complex 节点并持续处理定时发布事件。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomComplexNode>());
  rclcpp::shutdown();
  return 0;
}
