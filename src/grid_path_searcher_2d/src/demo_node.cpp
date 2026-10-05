/**
 * @file demo_node.cpp
 * @brief 二维 A*、TimeBreak A*、Dijkstra、JPS、PRM 与五种 RRT 系算法演示节点。
 *
 * 文件职责：接收随机 OccupancyGrid 和 waypoint_generator 的目标，调用
 * GridSearcher2D 完成十种算法搜索，并将每个障碍栅格作为独立方形像素发布，
 * 使用 steady_clock 分别统计十种算法的纯计算时间，同时向 RViz2 发布访问
 * 节点、起终点和规划路径。
 *
 * 包含的类：PlannerDemoNode，负责参数、话题连接、规划和 Marker 生成。
 * 包含的函数：PlannerDemoNode()、onMap()、onWaypoints()、makeMarker()、
 * makeMapPixels()、makeEndpoints()、makeRoadmapMarker()、makeRrtTreeMarker()、
 * makeRrtStarTreeMarker()、
 * publishResult()、publishTimedAStarResult()、publishJpsResult()、
 * publishPrmResult()、publishRrtResult()、publishRrtStarResult()、planTo() 和 main()。
 */
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "grid_path_searcher_2d/grid_searcher_2d.hpp"

using grid_path_searcher_2d::GridIndex;
using grid_path_searcher_2d::GridEdge;
using grid_path_searcher_2d::GridSearcher2D;
using grid_path_searcher_2d::SearchResult;

class PlannerDemoNode : public rclcpp::Node
{
public:
  /**
   * 初始化步骤：读取起点/默认终点与占据参数，创建持久化可视化发布器，订阅
   * 随机地图和航点；地图首次到达后可自动执行默认规划。
   */
  PlannerDemoNode() : Node("demo_node_2d")
  {
    start_x_ = declare_parameter("planning.start_x", -8.0);
    start_y_ = declare_parameter("planning.start_y", -8.0);
    default_goal_x_ = declare_parameter("planning.goal_x", 8.0);
    default_goal_y_ = declare_parameter("planning.goal_y", 8.0);
    auto_demo_ = declare_parameter("planning.auto_demo", true);
    occupied_threshold_ = declare_parameter("map.occupied_threshold", 50);
    allow_unknown_ = declare_parameter("map.allow_unknown", false);
    astar_heuristic_weight_ = declare_parameter("astar.heuristic_weight", 1.0);
    timed_astar_heuristic_weight_ =
      declare_parameter("timed_astar.heuristic_weight", 1.0);
    jps_heuristic_weight_ = declare_parameter("jps.heuristic_weight", 1.0);
    prm_sample_count_ = declare_parameter("prm.sample_count", 500);
    prm_k_neighbors_ = declare_parameter("prm.k_neighbors", 15);
    prm_seed_ = declare_parameter("prm.seed", 23);
    rrt_max_iterations_ = declare_parameter("rrt.max_iterations", 8000);
    rrt_step_size_ = declare_parameter("rrt.step_size", 5.0);
    rrt_goal_bias_ = declare_parameter("rrt.goal_bias", 0.12);
    rrt_seed_ = declare_parameter("rrt.seed", 31);
    rrt_star_max_iterations_ = declare_parameter("rrt_star.max_iterations", 5000);
    rrt_star_step_size_ = declare_parameter("rrt_star.step_size", 5.0);
    rrt_star_rewire_radius_ = declare_parameter("rrt_star.rewire_radius", 12.0);
    rrt_star_goal_bias_ = declare_parameter("rrt_star.goal_bias", 0.12);
    rrt_star_seed_ = declare_parameter("rrt_star.seed", 37);
    kinodynamic_max_iterations_ = declare_parameter("kinodynamic_rrt_star.max_iterations", 6000);
    kinodynamic_time_step_ = declare_parameter("kinodynamic_rrt_star.time_step", 0.8);
    kinodynamic_max_speed_ = declare_parameter("kinodynamic_rrt_star.max_speed", 6.0);
    kinodynamic_max_acceleration_ =
      declare_parameter("kinodynamic_rrt_star.max_acceleration", 4.0);
    kinodynamic_rewire_radius_ =
      declare_parameter("kinodynamic_rrt_star.rewire_radius", 10.0);
    kinodynamic_goal_bias_ = declare_parameter("kinodynamic_rrt_star.goal_bias", 0.15);
    kinodynamic_seed_ = declare_parameter("kinodynamic_rrt_star.seed", 41);
    anytime_max_iterations_ = declare_parameter("anytime_rrt_star.max_iterations", 12000);
    anytime_time_budget_ms_ = declare_parameter("anytime_rrt_star.time_budget_ms", 200.0);
    anytime_step_size_ = declare_parameter("anytime_rrt_star.step_size", 5.0);
    anytime_rewire_radius_ = declare_parameter("anytime_rrt_star.rewire_radius", 12.0);
    anytime_goal_bias_ = declare_parameter("anytime_rrt_star.goal_bias", 0.12);
    anytime_seed_ = declare_parameter("anytime_rrt_star.seed", 43);
    informed_max_iterations_ = declare_parameter("informed_rrt_star.max_iterations", 7000);
    informed_step_size_ = declare_parameter("informed_rrt_star.step_size", 5.0);
    informed_rewire_radius_ = declare_parameter("informed_rrt_star.rewire_radius", 12.0);
    informed_goal_bias_ = declare_parameter("informed_rrt_star.goal_bias", 0.12);
    informed_seed_ = declare_parameter("informed_rrt_star.seed", 47);
    timed_astar_limit_ms_ = declare_parameter("timed_astar.time_limit_ms", 2.0);

    // RViz 参数面板修改参数后，此回调校验并更新运行时成员；定时器随后自动重规划。
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&PlannerDemoNode::onParametersChanged, this, std::placeholders::_1));
    parameter_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&PlannerDemoNode::replanIfRequested, this));

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/grid_map_vis", latched_qos);
    astar_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/astar_path", latched_qos);
    timed_astar_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/timed_astar_path", latched_qos);
    dijkstra_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/dijkstra_path", latched_qos);
    jps_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/jps_path", latched_qos);
    prm_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/prm_path", latched_qos);
    rrt_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/rrt_path", latched_qos);
    rrt_star_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/rrt_star_path", latched_qos);
    kinodynamic_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/kinodynamic_rrt_star_path", latched_qos);
    anytime_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/anytime_rrt_star_path", latched_qos);
    informed_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/informed_rrt_star_path", latched_qos);
    astar_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/astar_path_vis", latched_qos);
    timed_astar_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/timed_astar_path_vis", latched_qos);
    timed_astar_visited_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/timed_astar_visited_vis", latched_qos);
    dijkstra_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/dijkstra_path_vis", latched_qos);
    jps_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/jps_path_vis", latched_qos);
    jump_points_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/jps_jump_points_vis", latched_qos);
    prm_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/prm_path_vis", latched_qos);
    prm_roadmap_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/prm_roadmap_vis", latched_qos);
    rrt_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/rrt_path_vis", latched_qos);
    rrt_tree_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/rrt_tree_vis", latched_qos);
    rrt_star_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/rrt_star_path_vis", latched_qos);
    rrt_star_tree_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/rrt_star_tree_vis", latched_qos);
    kinodynamic_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/kinodynamic_rrt_star_path_vis", latched_qos);
    kinodynamic_tree_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/kinodynamic_rrt_star_tree_vis", latched_qos);
    anytime_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/anytime_rrt_star_path_vis", latched_qos);
    anytime_tree_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/anytime_rrt_star_tree_vis", latched_qos);
    informed_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/informed_rrt_star_path_vis", latched_qos);
    informed_tree_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/informed_rrt_star_tree_vis", latched_qos);
    visited_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/visited_nodes_vis", latched_qos);
    map_pixels_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/map_pixels_vis", latched_qos);
    endpoints_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/endpoints_vis", latched_qos);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/random_map_2d/global_map", latched_qos,
      std::bind(&PlannerDemoNode::onMap, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoint_generator/waypoints", 10,
      std::bind(&PlannerDemoNode::onWaypoints, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Waiting for /random_map_2d/global_map ...");
  }

private:
  /**
   * 接收 RViz 面板的全部动态参数：先检查范围，再更新对应算法成员，并设置
   * 重规划标志。真正耗时的规划由定时器在参数服务回调结束后执行。
   */
  rcl_interfaces::msg::SetParametersResult onParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "map.occupied_threshold") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 1 || value > 100) {
          result.successful = false;
          result.reason = "map.occupied_threshold must be in [1, 100]";
          return result;
        }
        occupied_threshold_ = value;
      } else if (name == "map.allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } else if (name == "astar.heuristic_weight") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 5.0) {
          result.successful = false;
          result.reason = "astar.heuristic_weight must be in [0, 5]";
          return result;
        }
        astar_heuristic_weight_ = value;
      } else if (name == "timed_astar.heuristic_weight") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 5.0) {
          result.successful = false;
          result.reason = "timed_astar.heuristic_weight must be in [0, 5]";
          return result;
        }
        timed_astar_heuristic_weight_ = value;
      } else if (name == "timed_astar.time_limit_ms") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "timed_astar.time_limit_ms must be > 0";
          return result;
        }
        timed_astar_limit_ms_ = value;
      } else if (name == "jps.heuristic_weight") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 5.0) {
          result.successful = false;
          result.reason = "jps.heuristic_weight must be in [0, 5]";
          return result;
        }
        jps_heuristic_weight_ = value;
      } else if (name == "prm.sample_count") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 10) {
          result.successful = false;
          result.reason = "prm.sample_count must be >= 10";
          return result;
        }
        prm_sample_count_ = value;
      } else if (name == "prm.k_neighbors") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 1) {
          result.successful = false;
          result.reason = "prm.k_neighbors must be >= 1";
          return result;
        }
        prm_k_neighbors_ = value;
      } else if (name == "prm.seed") {
        prm_seed_ = static_cast<int>(parameter.as_int());
      } else if (name == "rrt.max_iterations") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 100) {
          result.successful = false;
          result.reason = "rrt.max_iterations must be >= 100";
          return result;
        }
        rrt_max_iterations_ = value;
      } else if (name == "rrt.step_size") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "rrt.step_size must be > 0";
          return result;
        }
        rrt_step_size_ = value;
      } else if (name == "rrt.goal_bias") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 1.0) {
          result.successful = false;
          result.reason = "rrt.goal_bias must be in [0, 1]";
          return result;
        }
        rrt_goal_bias_ = value;
      } else if (name == "rrt.seed") {
        rrt_seed_ = static_cast<int>(parameter.as_int());
      } else if (name == "rrt_star.max_iterations") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 100) {
          result.successful = false;
          result.reason = "rrt_star.max_iterations must be >= 100";
          return result;
        }
        rrt_star_max_iterations_ = value;
      } else if (name == "rrt_star.step_size") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "rrt_star.step_size must be > 0";
          return result;
        }
        rrt_star_step_size_ = value;
      } else if (name == "rrt_star.rewire_radius") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "rrt_star.rewire_radius must be > 0";
          return result;
        }
        rrt_star_rewire_radius_ = value;
      } else if (name == "rrt_star.goal_bias") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 1.0) {
          result.successful = false;
          result.reason = "rrt_star.goal_bias must be in [0, 1]";
          return result;
        }
        rrt_star_goal_bias_ = value;
      } else if (name == "rrt_star.seed") {
        rrt_star_seed_ = static_cast<int>(parameter.as_int());
      } else if (name == "kinodynamic_rrt_star.max_iterations") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 100) {
          result.successful = false;
          result.reason = "kinodynamic_rrt_star.max_iterations must be >= 100";
          return result;
        }
        kinodynamic_max_iterations_ = value;
      } else if (name == "kinodynamic_rrt_star.time_step") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "kinodynamic_rrt_star.time_step must be > 0";
          return result;
        }
        kinodynamic_time_step_ = value;
      } else if (name == "kinodynamic_rrt_star.max_speed") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "kinodynamic_rrt_star.max_speed must be > 0";
          return result;
        }
        kinodynamic_max_speed_ = value;
      } else if (name == "kinodynamic_rrt_star.max_acceleration") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "kinodynamic_rrt_star.max_acceleration must be > 0";
          return result;
        }
        kinodynamic_max_acceleration_ = value;
      } else if (name == "kinodynamic_rrt_star.rewire_radius") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "kinodynamic_rrt_star.rewire_radius must be > 0";
          return result;
        }
        kinodynamic_rewire_radius_ = value;
      } else if (name == "kinodynamic_rrt_star.goal_bias") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 1.0) {
          result.successful = false;
          result.reason = "kinodynamic_rrt_star.goal_bias must be in [0, 1]";
          return result;
        }
        kinodynamic_goal_bias_ = value;
      } else if (name == "kinodynamic_rrt_star.seed") {
        kinodynamic_seed_ = static_cast<int>(parameter.as_int());
      } else if (name == "anytime_rrt_star.max_iterations") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 100) {
          result.successful = false;
          result.reason = "anytime_rrt_star.max_iterations must be >= 100";
          return result;
        }
        anytime_max_iterations_ = value;
      } else if (name == "anytime_rrt_star.time_budget_ms") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "anytime_rrt_star.time_budget_ms must be > 0";
          return result;
        }
        anytime_time_budget_ms_ = value;
      } else if (name == "anytime_rrt_star.step_size") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "anytime_rrt_star.step_size must be > 0";
          return result;
        }
        anytime_step_size_ = value;
      } else if (name == "anytime_rrt_star.rewire_radius") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "anytime_rrt_star.rewire_radius must be > 0";
          return result;
        }
        anytime_rewire_radius_ = value;
      } else if (name == "anytime_rrt_star.goal_bias") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 1.0) {
          result.successful = false;
          result.reason = "anytime_rrt_star.goal_bias must be in [0, 1]";
          return result;
        }
        anytime_goal_bias_ = value;
      } else if (name == "anytime_rrt_star.seed") {
        anytime_seed_ = static_cast<int>(parameter.as_int());
      } else if (name == "informed_rrt_star.max_iterations") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 100) {
          result.successful = false;
          result.reason = "informed_rrt_star.max_iterations must be >= 100";
          return result;
        }
        informed_max_iterations_ = value;
      } else if (name == "informed_rrt_star.step_size") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "informed_rrt_star.step_size must be > 0";
          return result;
        }
        informed_step_size_ = value;
      } else if (name == "informed_rrt_star.rewire_radius") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "informed_rrt_star.rewire_radius must be > 0";
          return result;
        }
        informed_rewire_radius_ = value;
      } else if (name == "informed_rrt_star.goal_bias") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > 1.0) {
          result.successful = false;
          result.reason = "informed_rrt_star.goal_bias must be in [0, 1]";
          return result;
        }
        informed_goal_bias_ = value;
      } else if (name == "informed_rrt_star.seed") {
        informed_seed_ = static_cast<int>(parameter.as_int());
      } else {
        continue;
      }
      replan_requested_ = true;
    }
    return result;
  }

  /** 参数回调只置位；本定时器在节点执行循环中用当前目标安全地重新规划。 */
  void replanIfRequested()
  {
    if (!replan_requested_ || !map_ready_ || !have_goal_) {return;}
    replan_requested_ = false;
    // 占据阈值或未知格策略可能已经改变，重规划前立即刷新搜索器中的地图规则。
    if (!searcher_.setMap(map_, occupied_threshold_, allow_unknown_)) {
      RCLCPP_ERROR(get_logger(), "Cannot apply updated map traversal parameters");
      return;
    }
    RCLCPP_INFO(get_logger(), "Planning parameters changed; replanning current goal");
    planTo(last_goal_x_, last_goal_y_);
  }

  /** 缓存并转发地图；第一次收到合法地图时根据 auto_demo 执行默认目标规划。 */
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    const bool first_map = !map_ready_;
    if (!searcher_.setMap(*message, occupied_threshold_, allow_unknown_)) {
      RCLCPP_ERROR(get_logger(), "Received an invalid OccupancyGrid");
      return;
    }
    map_ = *message;
    map_ready_ = true;
    map_pub_->publish(map_);
    map_pixels_pub_->publish(makeMapPixels());
    if (first_map) {
      RCLCPP_INFO(
        get_logger(), "Received 2D map: %u x %u cells", map_.info.width, map_.info.height);
    }
    if (auto_demo_ && !auto_demo_done_) {
      auto_demo_done_ = true;
      planTo(default_goal_x_, default_goal_y_);
    }
  }

  /**
   * 遍历 OccupancyGrid，把每个占据单元的中心加入 CUBE_LIST。方块边长略小于
   * 地图分辨率，因此相邻障碍之间保留细缝，能清楚看到一个栅格一个像素。
   */
  visualization_msgs::msg::Marker makeMapPixels() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = "occupied_pixels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_.info.resolution * 0.88;
    marker.scale.y = map_.info.resolution * 0.88;
    marker.scale.z = std::max(0.04, static_cast<double>(map_.info.resolution) * 0.20);
    marker.color.a = 1.0F;
    marker.color.r = 0.08F;
    marker.color.g = 0.12F;
    marker.color.b = 0.18F;
    for (int y = 0; y < static_cast<int>(map_.info.height); ++y) {
      for (int x = 0; x < static_cast<int>(map_.info.width); ++x) {
        const auto offset = static_cast<std::size_t>(
          y * static_cast<int>(map_.info.width) + x);
        if (map_.data[offset] < occupied_threshold_) {continue;}
        auto point = searcher_.gridToWorld({x, y});
        point.z = marker.scale.z * 0.5;
        marker.points.push_back(point);
      }
    }
    return marker;
  }

  /** 创建起点和终点像素：绿色代表起点，紫色代表当前终点。 */
  visualization_msgs::msg::Marker makeEndpoints(
    const GridIndex & start, const GridIndex & goal) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = "start_and_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // 起点和终点保持普通栅格尺寸，不用放大的方块遮挡周围单元。
    marker.scale.x = map_.info.resolution * 0.88;
    marker.scale.y = map_.info.resolution * 0.88;
    marker.scale.z = std::max(0.04, static_cast<double>(map_.info.resolution) * 0.20);
    auto start_point = searcher_.gridToWorld(start);
    auto goal_point = searcher_.gridToWorld(goal);
    start_point.z = goal_point.z = marker.scale.z * 0.5;
    marker.points = {start_point, goal_point};
    std_msgs::msg::ColorRGBA start_color;
    start_color.r = 0.0F;
    start_color.g = 1.0F;
    start_color.b = 0.2F;
    start_color.a = 1.0F;
    std_msgs::msg::ColorRGBA goal_color;
    goal_color.r = 0.85F;
    goal_color.g = 0.0F;
    goal_color.b = 1.0F;
    goal_color.a = 1.0F;
    marker.colors = {start_color, goal_color};
    return marker;
  }

  /** 取 Path 的最后一个航点作为新目标并触发重新规划。 */
  void onWaypoints(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!map_ready_ || message->poses.empty()) {return;}
    const auto & position = message->poses.back().pose.position;
    planTo(position.x, position.y);
  }

  /** 把栅格序列转换为 POINTS 或 LINE_STRIP Marker，并设置颜色与尺寸。 */
  visualization_msgs::msg::Marker makeMarker(
    const std::vector<GridIndex> & cells, const std::string & name_space,
    const int type, const float red, const float green, const float blue,
    const double scale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = name_space;
    marker.id = 0;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = type == visualization_msgs::msg::Marker::CUBE_LIST ?
      std::max(0.04, static_cast<double>(map_.info.resolution) * 0.20) : scale;
    marker.color.a = 1.0F;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.points.reserve(cells.size());
    for (const auto & cell : cells) {
      auto point = searcher_.gridToWorld(cell);
      if (type == visualization_msgs::msg::Marker::POINTS) {
        point.z = 0.03;
      } else if (type == visualization_msgs::msg::Marker::CUBE_LIST) {
        point.z = marker.scale.z * 0.5;
      } else {
        point.z = 0.10;
      }
      marker.points.push_back(point);
    }
    return marker;
  }

  /** 把 PRM 的无碰撞连接边转换为灰色 LINE_LIST，显示随机路线图结构。 */
  visualization_msgs::msg::Marker makeRoadmapMarker(
    const std::vector<GridEdge> & edges) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = "prm_roadmap";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.012;
    marker.color.r = 0.42F;
    marker.color.g = 0.42F;
    marker.color.b = 0.46F;
    marker.color.a = 0.38F;
    marker.points.reserve(edges.size() * 2U);
    for (const auto & edge : edges) {
      auto from = searcher_.gridToWorld(edge.from);
      auto to = searcher_.gridToWorld(edge.to);
      from.z = to.z = 0.055;
      marker.points.push_back(from);
      marker.points.push_back(to);
    }
    return marker;
  }

  /** 把 RRT 的父子边转换为棕色 LINE_LIST，显示搜索树从起点向外生长的结构。 */
  visualization_msgs::msg::Marker makeRrtTreeMarker(
    const std::vector<GridEdge> & edges) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = "rrt_tree";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.016;
    marker.color.r = 0.62F;
    marker.color.g = 0.34F;
    marker.color.b = 0.08F;
    marker.color.a = 0.62F;
    marker.points.reserve(edges.size() * 2U);
    for (const auto & edge : edges) {
      auto from = searcher_.gridToWorld(edge.from);
      auto to = searcher_.gridToWorld(edge.to);
      from.z = to.z = 0.07;
      marker.points.push_back(from);
      marker.points.push_back(to);
    }
    return marker;
  }

  /** 把 RRT* 重连后的最终树转换为蓝绿色 LINE_LIST。 */
  visualization_msgs::msg::Marker makeRrtStarTreeMarker(
    const std::vector<GridEdge> & edges) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = "rrt_star_tree";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.014;
    marker.color.r = 0.05F;
    marker.color.g = 0.48F;
    marker.color.b = 0.58F;
    marker.color.a = 0.52F;
    marker.points.reserve(edges.size() * 2U);
    for (const auto & edge : edges) {
      auto from = searcher_.gridToWorld(edge.from);
      auto to = searcher_.gridToWorld(edge.to);
      from.z = to.z = 0.075;
      marker.points.push_back(from);
      marker.points.push_back(to);
    }
    return marker;
  }

  /** 为新增 RRT* 变体创建可区分颜色和高度的搜索树 LINE_LIST。 */
  visualization_msgs::msg::Marker makeVariantTreeMarker(
    const std::vector<GridEdge> & edges, const std::string & name_space,
    const float red, const float green, const float blue, const double height) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    marker.header.stamp = now();
    marker.ns = name_space;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.014;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 0.52F;
    marker.points.reserve(edges.size() * 2U);
    for (const auto & edge : edges) {
      auto from = searcher_.gridToWorld(edge.from);
      auto to = searcher_.gridToWorld(edge.to);
      from.z = to.z = height;
      marker.points.push_back(from);
      marker.points.push_back(to);
    }
    return marker;
  }

  /** 将一个 RRT* 变体的路径和树发布到它自己的话题。 */
  void publishRrtVariant(
    const SearchResult & result, const std::string & algorithm_name,
    const std::string & marker_namespace,
    const float path_red, const float path_green, const float path_blue,
    const float tree_red, const float tree_green, const float tree_blue,
    const double tree_height,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & path_publisher,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_publisher,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & tree_publisher)
  {
    tree_publisher->publish(makeVariantTreeMarker(
      result.tree_edges, marker_namespace + "_tree",
      tree_red, tree_green, tree_blue, tree_height));
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "%s could not reach goal", algorithm_name.c_str());
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_publisher->publish(path);
    marker_publisher->publish(makeMarker(
      result.path, marker_namespace + "_path", visualization_msgs::msg::Marker::LINE_STRIP,
      path_red, path_green, path_blue,
      std::max(0.07, static_cast<double>(map_.info.resolution) * 0.50)));
    RCLCPP_INFO(
      get_logger(), "%s success: cost=%.3f m, path nodes=%zu, tree edges=%zu",
      algorithm_name.c_str(), result.cost, result.path.size(), result.tree_edges.size());
  }

  /** 将搜索结果转换成 Path 和彩色 Marker；失败时输出明确警告。 */
  void publishResult(
    const SearchResult & result, const bool astar,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & path_publisher,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & marker_publisher)
  {
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "%s could not find a path", astar ? "A*" : "Dijkstra");
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    path.poses.reserve(result.path.size());
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_publisher->publish(path);
    marker_publisher->publish(makeMarker(
      result.path, astar ? "astar_path" : "dijkstra_path",
      visualization_msgs::msg::Marker::LINE_STRIP,
      astar ? 1.0F : 0.1F, astar ? 0.1F : 0.4F, astar ? 0.1F : 1.0F,
      std::max(0.06, static_cast<double>(map_.info.resolution) * 0.45)));
    RCLCPP_INFO(
      get_logger(), "%s success: cost=%.3f m, path=%zu cells, visited=%zu cells",
      astar ? "A*" : "Dijkstra", result.cost, result.path.size(), result.visited.size());
  }

  /** 发布 TimeBreak A* 的粉红路径和紫色访问节点，并明确报告成功或超时。 */
  void publishTimedAStarResult(const SearchResult & result)
  {
    timed_astar_visited_pub_->publish(makeMarker(
      result.visited, "timed_astar_visited", visualization_msgs::msg::Marker::POINTS,
      0.72F, 0.20F, 1.0F,
      std::max(0.025, static_cast<double>(map_.info.resolution) * 0.22)));
    if (!result.success) {
      visualization_msgs::msg::Marker clear;
      clear.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
      clear.header.stamp = now();
      clear.ns = "timed_astar_path";
      clear.id = 0;
      clear.action = visualization_msgs::msg::Marker::DELETE;
      timed_astar_marker_pub_->publish(clear);
      nav_msgs::msg::Path empty_path;
      empty_path.header = clear.header;
      timed_astar_path_pub_->publish(empty_path);
      if (result.timed_out) {
        RCLCPP_WARN(
          get_logger(), "TimeBreak A* timed out at %.3f ms after visiting %zu cells",
          timed_astar_limit_ms_, result.visited.size());
      } else {
        RCLCPP_WARN(get_logger(), "TimeBreak A* could not find a path");
      }
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    timed_astar_path_pub_->publish(path);
    timed_astar_marker_pub_->publish(makeMarker(
      result.path, "timed_astar_path", visualization_msgs::msg::Marker::LINE_STRIP,
      1.0F, 0.32F, 0.62F,
      std::max(0.06, static_cast<double>(map_.info.resolution) * 0.45)));
    RCLCPP_INFO(
      get_logger(), "TimeBreak A* success: cost=%.3f m, path=%zu, visited=%zu, limit=%.3f ms",
      result.cost, result.path.size(), result.visited.size(), timed_astar_limit_ms_);
  }

  /** 发布绿色 JPS 折线路径以及亮青色、普通栅格大小的关键跳点。 */
  void publishJpsResult(const SearchResult & result)
  {
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "JPS could not find a path");
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    jps_path_pub_->publish(path);
    jps_marker_pub_->publish(makeMarker(
      result.path, "jps_path", visualization_msgs::msg::Marker::LINE_STRIP,
      0.1F, 0.95F, 0.2F,
      std::max(0.06, static_cast<double>(map_.info.resolution) * 0.45)));
    jump_points_pub_->publish(makeMarker(
      result.jump_points, "jps_jump_points", visualization_msgs::msg::Marker::CUBE_LIST,
      0.0F, 1.0F, 1.0F, map_.info.resolution * 0.88));
    RCLCPP_INFO(
      get_logger(), "JPS success: cost=%.3f m, jump points=%zu, visited=%zu",
      result.cost, result.jump_points.size(), result.visited.size());
  }

  /** 发布灰色 PRM 路线图和洋红色最终路径；失败时仍保留路线图帮助观察。 */
  void publishPrmResult(const SearchResult & result)
  {
    prm_roadmap_pub_->publish(makeRoadmapMarker(result.roadmap_edges));
    if (!result.success) {
      RCLCPP_WARN(
        get_logger(), "PRM could not connect start and goal (samples=%d, k=%d)",
        prm_sample_count_, prm_k_neighbors_);
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    prm_path_pub_->publish(path);
    prm_marker_pub_->publish(makeMarker(
      result.path, "prm_path", visualization_msgs::msg::Marker::LINE_STRIP,
      1.0F, 0.1F, 0.82F,
      std::max(0.06, static_cast<double>(map_.info.resolution) * 0.45)));
    RCLCPP_INFO(
      get_logger(), "PRM success: cost=%.3f m, path nodes=%zu, roadmap edges=%zu, visited=%zu",
      result.cost, result.path.size(), result.roadmap_edges.size(), result.visited.size());
  }

  /** 发布棕色 RRT 搜索树和橙色最终路径；失败时保留已生长的树。 */
  void publishRrtResult(const SearchResult & result)
  {
    rrt_tree_pub_->publish(makeRrtTreeMarker(result.tree_edges));
    if (!result.success) {
      RCLCPP_WARN(
        get_logger(), "RRT could not reach goal (iterations=%d, step=%.2f, bias=%.2f)",
        rrt_max_iterations_, rrt_step_size_, rrt_goal_bias_);
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    rrt_path_pub_->publish(path);
    rrt_marker_pub_->publish(makeMarker(
      result.path, "rrt_path", visualization_msgs::msg::Marker::LINE_STRIP,
      1.0F, 0.48F, 0.0F,
      std::max(0.06, static_cast<double>(map_.info.resolution) * 0.45)));
    RCLCPP_INFO(
      get_logger(), "RRT success: cost=%.3f m, path nodes=%zu, tree edges=%zu",
      result.cost, result.path.size(), result.tree_edges.size());
  }

  /** 发布蓝绿色 RRT* 重连树和亮黄色优化路径。 */
  void publishRrtStarResult(const SearchResult & result)
  {
    rrt_star_tree_pub_->publish(makeRrtStarTreeMarker(result.tree_edges));
    if (!result.success) {
      RCLCPP_WARN(
        get_logger(), "RRT* could not reach goal (iterations=%d, radius=%.2f)",
        rrt_star_max_iterations_, rrt_star_rewire_radius_);
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & cell : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = searcher_.gridToWorld(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    rrt_star_path_pub_->publish(path);
    rrt_star_marker_pub_->publish(makeMarker(
      result.path, "rrt_star_path", visualization_msgs::msg::Marker::LINE_STRIP,
      1.0F, 0.92F, 0.05F,
      std::max(0.07, static_cast<double>(map_.info.resolution) * 0.50)));
    RCLCPP_INFO(
      get_logger(), "RRT* success: cost=%.3f m, path nodes=%zu, final tree edges=%zu",
      result.cost, result.path.size(), result.tree_edges.size());
  }

  /**
   * 规划步骤：世界坐标转栅格 → 执行十种算法 → 发布搜索结果、JPS 跳点、
   * PRM 路线图，以及各 RRT/RRT* 变体的搜索树。
   */
  void planTo(const double goal_x, const double goal_y)
  {
    GridIndex start;
    GridIndex goal;
    if (!searcher_.worldToGrid(start_x_, start_y_, start)) {
      RCLCPP_WARN(get_logger(), "Configured start is outside the map");
      return;
    }
    if (!searcher_.worldToGrid(goal_x, goal_y, goal)) {
      RCLCPP_WARN(get_logger(), "Goal (%.2f, %.2f) is outside the map", goal_x, goal_y);
      return;
    }
    last_goal_x_ = goal_x;
    last_goal_y_ = goal_y;
    have_goal_ = true;
    endpoints_pub_->publish(makeEndpoints(start, goal));
    // 每种算法单独计时；计时区间只包围搜索调用，不包含 Marker/Path 消息发布。
    const auto astar_begin = std::chrono::steady_clock::now();
    // true：启用 h，运行 A*。
    const auto astar_result = searcher_.searchAStar(start, goal, astar_heuristic_weight_);
    const double astar_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - astar_begin).count();

    const auto timed_astar_begin = std::chrono::steady_clock::now();
    const auto timed_astar_result = searcher_.searchTimedAStar(
      start, goal, timed_astar_limit_ms_, timed_astar_heuristic_weight_);
    const double timed_astar_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - timed_astar_begin).count();

    const auto dijkstra_begin = std::chrono::steady_clock::now();
    const auto dijkstra_result = searcher_.searchDijkstra(start, goal);
    const double dijkstra_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - dijkstra_begin).count();

    const auto jps_begin = std::chrono::steady_clock::now();
    const auto jps_result = searcher_.searchJps(start, goal, jps_heuristic_weight_);
    const double jps_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - jps_begin).count();

    const auto prm_begin = std::chrono::steady_clock::now();
    const auto prm_result = searcher_.searchPrm(
      start, goal, prm_sample_count_, prm_k_neighbors_, prm_seed_);
    const double prm_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - prm_begin).count();

    const auto rrt_begin = std::chrono::steady_clock::now();
    const auto rrt_result = searcher_.searchRrt(
      start, goal, rrt_max_iterations_, rrt_step_size_, rrt_goal_bias_, rrt_seed_);
    const double rrt_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - rrt_begin).count();

    const auto rrt_star_begin = std::chrono::steady_clock::now();
    const auto rrt_star_result = searcher_.searchRrtStar(
      start, goal, rrt_star_max_iterations_, rrt_star_step_size_,
      rrt_star_rewire_radius_, rrt_star_goal_bias_, rrt_star_seed_);
    const double rrt_star_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - rrt_star_begin).count();

    const auto kinodynamic_begin = std::chrono::steady_clock::now();
    const auto kinodynamic_result = searcher_.searchKinodynamicRrtStar(
      start, goal, kinodynamic_max_iterations_, kinodynamic_time_step_,
      kinodynamic_max_speed_, kinodynamic_max_acceleration_,
      kinodynamic_rewire_radius_, kinodynamic_goal_bias_, kinodynamic_seed_);
    const double kinodynamic_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - kinodynamic_begin).count();

    const auto anytime_begin = std::chrono::steady_clock::now();
    const auto anytime_result = searcher_.searchAnytimeRrtStar(
      start, goal, anytime_max_iterations_, anytime_time_budget_ms_,
      anytime_step_size_, anytime_rewire_radius_, anytime_goal_bias_, anytime_seed_);
    const double anytime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - anytime_begin).count();

    const auto informed_begin = std::chrono::steady_clock::now();
    const auto informed_result = searcher_.searchInformedRrtStar(
      start, goal, informed_max_iterations_, informed_step_size_,
      informed_rewire_radius_, informed_goal_bias_, informed_seed_);
    const double informed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - informed_begin).count();

    RCLCPP_INFO(get_logger(), "A* computation time: %.3f ms", astar_ms);
    RCLCPP_INFO(
      get_logger(), "TimeBreak A* computation time: %.3f ms (limit %.3f ms, %s)",
      timed_astar_ms, timed_astar_limit_ms_, timed_astar_result.timed_out ? "timeout" : "finished");
    RCLCPP_INFO(get_logger(), "Dijkstra computation time: %.3f ms", dijkstra_ms);
    RCLCPP_INFO(get_logger(), "JPS computation time: %.3f ms", jps_ms);
    RCLCPP_INFO(
      get_logger(), "PRM computation time: %.3f ms (sampling + roadmap + graph search)",
      prm_ms);
    RCLCPP_INFO(
      get_logger(), "RRT computation time: %.3f ms (sampling + tree expansion)", rrt_ms);
    RCLCPP_INFO(
      get_logger(), "RRT* computation time: %.3f ms (sampling + choose parent + rewire)",
      rrt_star_ms);
    RCLCPP_INFO(
      get_logger(), "Kinodynamic-RRT* computation time: %.3f ms (state + control + rewire)",
      kinodynamic_ms);
    RCLCPP_INFO(
      get_logger(), "Anytime-RRT* computation time: %.3f ms (budget %.1f ms, %s)",
      anytime_ms, anytime_time_budget_ms_, anytime_result.timed_out ? "time budget" : "iterations");
    RCLCPP_INFO(
      get_logger(), "Informed RRT* computation time: %.3f ms (ellipse sampling + rewire)",
      informed_ms);
    visited_pub_->publish(makeMarker(
      astar_result.visited, "astar_visited", visualization_msgs::msg::Marker::POINTS,
      1.0F, 0.72F, 0.0F, std::max(0.025, static_cast<double>(map_.info.resolution) * 0.22)));
    publishResult(astar_result, true, astar_path_pub_, astar_marker_pub_);
    publishTimedAStarResult(timed_astar_result);
    publishResult(dijkstra_result, false, dijkstra_path_pub_, dijkstra_marker_pub_);
    publishJpsResult(jps_result);
    publishPrmResult(prm_result);
    publishRrtResult(rrt_result);
    publishRrtStarResult(rrt_star_result);
    publishRrtVariant(
      kinodynamic_result, "Kinodynamic-RRT*", "kinodynamic_rrt_star",
      0.10F, 0.95F, 1.00F, 0.04F, 0.55F, 0.72F, 0.080,
      kinodynamic_path_pub_, kinodynamic_marker_pub_, kinodynamic_tree_pub_);
    publishRrtVariant(
      anytime_result, "Anytime-RRT*", "anytime_rrt_star",
      1.00F, 0.42F, 0.78F, 0.62F, 0.18F, 0.48F, 0.085,
      anytime_path_pub_, anytime_marker_pub_, anytime_tree_pub_);
    publishRrtVariant(
      informed_result, "Informed RRT*", "informed_rrt_star",
      0.58F, 1.00F, 0.12F, 0.30F, 0.62F, 0.08F, 0.090,
      informed_path_pub_, informed_marker_pub_, informed_tree_pub_);
  }

  double start_x_{-8.0};
  double start_y_{-8.0};
  double default_goal_x_{8.0};
  double default_goal_y_{8.0};
  int occupied_threshold_{50};
  double astar_heuristic_weight_{1.0};
  double timed_astar_heuristic_weight_{1.0};
  double jps_heuristic_weight_{1.0};
  int prm_sample_count_{500};
  int prm_k_neighbors_{15};
  int prm_seed_{23};
  int rrt_max_iterations_{8000};
  int rrt_seed_{31};
  double rrt_step_size_{5.0};
  double rrt_goal_bias_{0.12};
  int rrt_star_max_iterations_{5000};
  int rrt_star_seed_{37};
  double rrt_star_step_size_{5.0};
  double rrt_star_rewire_radius_{12.0};
  double rrt_star_goal_bias_{0.12};
  int kinodynamic_max_iterations_{6000};
  int kinodynamic_seed_{41};
  double kinodynamic_time_step_{0.8};
  double kinodynamic_max_speed_{6.0};
  double kinodynamic_max_acceleration_{4.0};
  double kinodynamic_rewire_radius_{10.0};
  double kinodynamic_goal_bias_{0.15};
  int anytime_max_iterations_{12000};
  int anytime_seed_{43};
  double anytime_time_budget_ms_{200.0};
  double anytime_step_size_{5.0};
  double anytime_rewire_radius_{12.0};
  double anytime_goal_bias_{0.12};
  int informed_max_iterations_{7000};
  int informed_seed_{47};
  double informed_step_size_{5.0};
  double informed_rewire_radius_{12.0};
  double informed_goal_bias_{0.12};
  double timed_astar_limit_ms_{2.0};
  bool allow_unknown_{false};
  bool auto_demo_{true};
  bool auto_demo_done_{false};
  bool map_ready_{false};
  bool have_goal_{false};
  bool replan_requested_{false};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  nav_msgs::msg::OccupancyGrid map_;
  GridSearcher2D searcher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr parameter_timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr astar_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr timed_astar_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dijkstra_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr jps_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prm_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rrt_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rrt_star_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr kinodynamic_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr anytime_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr informed_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr astar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr timed_astar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr timed_astar_visited_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dijkstra_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr jps_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr jump_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr prm_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr prm_roadmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_star_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_star_tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kinodynamic_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kinodynamic_tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr anytime_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr anytime_tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr informed_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr informed_tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visited_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_pixels_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr endpoints_pub_;
};

/** 创建二维演示节点并进入 ROS 2 事件循环。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerDemoNode>());
  rclcpp::shutdown();
  return 0;
}
