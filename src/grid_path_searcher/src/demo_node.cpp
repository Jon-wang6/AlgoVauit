/**
 * @file demo_node.cpp
 * @brief 三维路径规划演示节点的入口与业务编排实现。
 *
 * 文件职责：接收 PointCloud2 障碍地图并写入三维占据栅格；接收 Path
 * 航点；依次运行 JPS 与 A*；把地图、已搜索节点和最终路径发布给 RViz2。
 *
 * 包含的类：
 * - TutorialDemoNode：三维演示 ROS 2 节点，管理订阅、规划和可视化发布。
 *
 * 包含的函数：
 * - TutorialDemoNode()：读取参数并建立搜索器、发布器和订阅器。
 * - onMap()：导入点云障碍，并在首次地图到达后触发默认演示。
 * - marker()：把一组三维坐标转换成 RViz Marker。
 * - planTo()：分别运行 JPS/A* 并发布搜索结果。
 * - onWaypoints()：读取最新目标航点并请求重新规划。
 * - main()：初始化 ROS 2、运行节点并在退出时清理资源。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "graph_searcher.hpp"

using Eigen::Vector3d;

// 三维教程规划节点：接收随机点云地图和目标航点，分别执行 JPS 与 A*，再发布 RViz 标记。
class TutorialDemoNode : public rclcpp::Node
{
public:
  /**
   * 初始化步骤：
   * 1. 读取地图尺寸、分辨率、起点和自动演示参数；
   * 2. 按地图边界创建三维栅格搜索器；
   * 3. 创建带 Transient Local 持久性的可视化发布器；
   * 4. 订阅随机点云地图和 waypoint_generator 输出的航点。
   */
  TutorialDemoNode() : Node("demo_node")
  {
    resolution_ = declare_parameter("map.resolution", 0.2);
    x_size_ = declare_parameter("map.x_size", 20.0);
    y_size_ = declare_parameter("map.y_size", 20.0);
    z_size_ = declare_parameter("map.z_size", 4.0);
    cloud_margin_ = declare_parameter("map.margin", 0.0);
    start_ << declare_parameter("planning.start_x", 0.0),
      declare_parameter("planning.start_y", 0.0),
      declare_parameter("planning.start_z", 1.0);
    auto_demo_ = declare_parameter("planning.auto_demo", true);

    const Vector3d lower(-x_size_ / 2.0, -y_size_ / 2.0, 0.0);
    const Vector3d upper(x_size_ / 2.0, y_size_ / 2.0, z_size_);
    path_finder_.initGridMap(
      resolution_, lower, upper,
      static_cast<int>(x_size_ / resolution_),
      static_cast<int>(y_size_ / resolution_),
      static_cast<int>(z_size_ / resolution_));

    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/grid_map_vis", latched_qos);
    path_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/grid_path_vis", latched_qos);
    closed_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/closed_nodes_vis", latched_qos);
    debug_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/debug_nodes_vis", latched_qos);

    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoint_generator/waypoints", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { onWaypoints(*msg); });
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/random_complex/global_map", latched_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onMap(*msg);});
    RCLCPP_INFO(get_logger(), "Waiting for /random_complex/global_map ...");
  }

private:
  /**
   * 首次收到 PointCloud2 地图时建立占据栅格。
   * 实现步骤：遍历 XYZ 点；按 margin 对每个点做三维膨胀；写入搜索器；
   * 转发地图给 RViz；最后根据 auto_demo 决定是否规划默认目标。
   */
  void onMap(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (map_ready_) {return;}
    const int inflate_xy = static_cast<int>(std::round(cloud_margin_ / resolution_));
    const int inflate_z = std::max(0, inflate_xy / 2);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    std::size_t count = 0;
    for (; x != x.end(); ++x, ++y, ++z) {
      for (int ix = -inflate_xy; ix <= inflate_xy; ++ix) {
        for (int iy = -inflate_xy; iy <= inflate_xy; ++iy) {
          for (int iz = -inflate_z; iz <= inflate_z; ++iz) {
            path_finder_.setObs(
              *x + ix * resolution_, *y + iy * resolution_, *z + iz * resolution_);
          }
        }
      }
      ++count;
    }
    map_pub_->publish(cloud);
    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "Received %zu points from /random_complex/global_map", count);
    if (auto_demo_) {planTo(Vector3d(7.0, 7.0, 1.0));}
  }

  /**
   * 将一组 Eigen 三维坐标转换为 RViz 的 CUBE_LIST Marker。
   * name 用来区分 A*、JPS 和关闭节点，RGBA 参数控制显示颜色。
   */
  visualization_msgs::msg::Marker marker(
    const std::vector<Vector3d> & nodes, const std::string & name,
    const float red, const float green, const float blue, const float alpha) const
  {
    visualization_msgs::msg::Marker result;
    result.header.frame_id = "world";
    result.header.stamp = now();
    result.ns = name;
    result.id = 0;
    result.type = visualization_msgs::msg::Marker::CUBE_LIST;
    result.action = visualization_msgs::msg::Marker::ADD;
    result.pose.orientation.w = 1.0;
    result.scale.x = resolution_;
    result.scale.y = resolution_;
    result.scale.z = resolution_;
    result.color.r = red;
    result.color.g = green;
    result.color.b = blue;
    result.color.a = alpha;
    result.points.reserve(nodes.size());
    for (const auto & node : nodes) {
      geometry_msgs::msg::Point point;
      point.x = node.x();
      point.y = node.y();
      point.z = node.z();
      result.points.push_back(point);
    }
    return result;
  }

  /**
   * 对一个目标点执行完整规划流程。
   * 1. 将目标高度限制在地图范围内；
   * 2. 运行 JPS，发布紫红色路径和调试节点；
   * 3. 清理本轮搜索状态；
   * 4. 运行普通 A*，发布绿色路径与蓝色扩展节点；
   * 5. 再次清理状态，使下一次点击目标时可重新规划。
   */
  void planTo(Vector3d target)
  {
    target.z() = std::clamp(target.z(), resolution_, z_size_ - resolution_);
    path_finder_.graphSearch(start_, target, true);
    auto jps_path = path_finder_.getPath();
    path_pub_->publish(marker(jps_path, "jps_path", 1.0F, 0.0F, 0.8F, 1.0F));
    debug_pub_->publish(marker(path_finder_.debugNodes, "debug_info", 0.0F, 0.0F, 0.0F, 0.4F));
    path_finder_.resetUsedGrids();

    path_finder_.graphSearch(start_, target, false);
    auto astar_path = path_finder_.getPath();
    path_pub_->publish(marker(astar_path, "astar_path", 0.0F, 1.0F, 0.0F, 1.0F));
    closed_pub_->publish(marker(
      path_finder_.getCloseNodes(), "closed_nodes", 0.0F, 0.2F, 1.0F, 0.45F));
    path_finder_.resetUsedGrids();
    RCLCPP_INFO(
      get_logger(), "goal=(%.2f, %.2f, %.2f), JPS nodes=%zu, A* nodes=%zu",
      target.x(), target.y(), target.z(), jps_path.size(), astar_path.size());
  }

  /** 接收航点路径，取第一个航点作为三维目标；地图尚未就绪时忽略请求。 */
  void onWaypoints(const nav_msgs::msg::Path & waypoints)
  {
    if (!map_ready_ || waypoints.poses.empty()) {
      return;
    }
    const auto & p = waypoints.poses.front().pose.position;
    planTo(Vector3d(p.x, p.y, p.z < 0.0 ? 1.0 : p.z));
  }

  double resolution_{};
  double x_size_{};
  double y_size_{};
  double z_size_{};
  double cloud_margin_{};
  bool auto_demo_{true};
  bool map_ready_{false};
  Vector3d start_{Vector3d::Zero()};
  gridPathFinder path_finder_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closed_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
};

/** ROS 2 程序入口：初始化通信、创建 demo_node、进入事件循环并在退出时清理。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TutorialDemoNode>());
  rclcpp::shutdown();
  return 0;
}
