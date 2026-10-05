/**
 * @file waypoint_generator.cpp
 * @brief 手动目标、预设轨迹和参数化分段轨迹的航点生成节点。
 *
 * 文件职责：接收 RViz 目标、里程计和轨迹触发消息，根据 waypoint_type
 * 选择单目标或预设图形，最终发布 nav_msgs/Path 与 PoseArray 可视化消息。
 *
 * 包含的类：
 * - WaypointGenerator：管理多种航点来源、轨迹变换、分段调度与发布。
 *
 * 包含的函数：
 * - WaypointGenerator()：读取参数并建立订阅器、发布器和定时器。
 * - odomCallback()/goalCallback()/triggerCallback()：处理三类输入。
 * - yawFromQuaternion()：从姿态四元数提取偏航角。
 * - refreshType()/stampPath()：刷新模式并补齐路径消息头。
 * - publishVisualization()/publishPath()：发布可视化和正式航点。
 * - selectedPattern()：选择 point/circle/eight 预设轨迹。
 * - getOrDeclare()：安全读取或声明动态参数。
 * - loadSegments()/publishDueSegment()：加载并按时间发布分段轨迹。
 * - main()：初始化 ROS 2、运行节点并完成退出清理。
 */
#include <cmath>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sample_waypoints.hpp"

// 航点生成节点：把 RViz 目标或预设轨迹转换为 nav_msgs/Path，供规划器订阅。
class WaypointGenerator : public rclcpp::Node
{
public:
  /** 读取航点模式和坐标系，创建里程计/目标/触发订阅器、两个发布器及分段定时器。 */
  WaypointGenerator() : Node("waypoint_generator")
  {
    waypoint_type_ = declare_parameter<std::string>("waypoint_type", "manual");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    declare_parameter<int>("segment_cnt", 0);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", 10, std::bind(&WaypointGenerator::odomCallback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/goal", 10, std::bind(&WaypointGenerator::goalCallback, this, std::placeholders::_1));
    trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/traj_start_trigger", 10,
      std::bind(&WaypointGenerator::triggerCallback, this, std::placeholders::_1));
    waypoints_pub_ = create_publisher<nav_msgs::msg::Path>("~/waypoints", 50);
    visualization_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/waypoints_vis", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&WaypointGenerator::publishDueSegment, this));
    RCLCPP_INFO(get_logger(), "Waypoint generator ready (type=%s)", waypoint_type_.c_str());
  }

private:
  /** 缓存最新里程计，用于相对轨迹和平移/旋转计算。 */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg;
    have_odom_ = true;
  }

  /** 将四元数转换为绕 Z 轴的偏航角。 */
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & message)
  {
    tf2::Quaternion quaternion;
    tf2::fromMsg(message, quaternion);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
  }

  /** 每次触发前重新读取 waypoint_type，允许运行时动态修改模式。 */
  void refreshType() {get_parameter("waypoint_type", waypoint_type_);}

  /** 给 Path 及其中每个 PoseStamped 填入统一时间戳和坐标系。 */
  void stampPath(nav_msgs::msg::Path & path)
  {
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    for (auto & pose : path.poses) {
      pose.header = path.header;
    }
  }

  /** 把里程计起点和全部航点转换为 PoseArray，供 RViz 预览。 */
  void publishVisualization(const nav_msgs::msg::Path & path)
  {
    geometry_msgs::msg::PoseArray poses;
    poses.header = path.header;
    if (have_odom_) {poses.poses.push_back(odom_.pose.pose);}
    for (const auto & pose : path.poses) {poses.poses.push_back(pose.pose);}
    visualization_pub_->publish(poses);
  }

  /** 发布航点的统一入口：补齐 header，先发可视化，再发正式 Path。 */
  void publishPath(nav_msgs::msg::Path path)
  {
    stampPath(path);
    publishVisualization(path);
    waypoints_pub_->publish(path);
  }

  /** 根据 waypoint_type 选择单点、圆形或八字形预设轨迹。 */
  nav_msgs::msg::Path selectedPattern() const
  {
    if (waypoint_type_ == "circle") {return waypoint_patterns::circle();}
    if (waypoint_type_ == "eight") {return waypoint_patterns::eight();}
    return waypoint_patterns::point();
  }

  /**
   * 处理 RViz 目标：
   * - point/circle/eight：发布预设轨迹；
   * - series：加载参数化分段轨迹；
   * - manual-lonely-waypoint：把本次点击直接作为唯一航点；
   * - 其他手动模式：按 Z 值分别执行添加、撤销和提交。
   */
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    refreshType();
    triggered_time_ = now();
    if (waypoint_type_ == "circle" || waypoint_type_ == "eight" || waypoint_type_ == "point") {
      publishPath(selectedPattern());
      return;
    }
    if (waypoint_type_ == "series") {
      loadSegments();
      return;
    }
    if (waypoint_type_ == "manual-lonely-waypoint") {
      if (msg->pose.position.z < 0.0) {
        RCLCPP_WARN(get_logger(), "Ignoring a goal below z=0 in manual-lonely-waypoint mode");
        return;
      }
      nav_msgs::msg::Path path;
      path.poses.push_back(*msg);
      publishPath(path);
      return;
    }

    if (msg->pose.position.z > 0.0) {
      auto pose = *msg;
      if (waypoint_type_ == "noyaw" && have_odom_) {
        pose.pose.orientation = waypoint_patterns::yawQuaternion(yawFromQuaternion(odom_.pose.pose.orientation));
      }
      pending_manual_.poses.push_back(pose);
      stampPath(pending_manual_);
      publishVisualization(pending_manual_);
    } else if (msg->pose.position.z > -1.0) {
      if (!pending_manual_.poses.empty()) {pending_manual_.poses.pop_back();}
      stampPath(pending_manual_);
      publishVisualization(pending_manual_);
    } else if (!pending_manual_.poses.empty()) {
      publishPath(pending_manual_);
      pending_manual_.poses.clear();
    }
  }

  /** 收到轨迹开始触发后，以当前里程计为基准发布预设或分段轨迹。 */
  void triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    if (!have_odom_) {
      RCLCPP_ERROR(get_logger(), "Cannot trigger a pattern before odometry is received");
      return;
    }
    refreshType();
    triggered_time_ = now();
    if (waypoint_type_ == "series") {loadSegments();} else {publishPath(selectedPattern());}
  }

  /** 若参数尚未声明则以默认值声明，然后返回参数当前值。 */
  template<typename T>
  T getOrDeclare(const std::string & name, const T & default_value)
  {
    if (!has_parameter(name)) {declare_parameter<T>(name, default_value);}
    T value = default_value;
    get_parameter(name, value);
    return value;
  }

  /**
   * 从 segN.* 参数读取分段轨迹，将局部 XYZ 按机器人当前偏航角旋转，
   * 再平移到当前里程计位置，并按 time_from_start 放入待发布队列。
   */
  void loadSegments()
  {
    segments_.clear();
    const int count = get_parameter("segment_cnt").as_int();
    const double base_yaw = have_odom_ ? yawFromQuaternion(odom_.pose.pose.orientation) : 0.0;
    for (int i = 0; i < count; ++i) {
      const std::string prefix = "seg" + std::to_string(i) + ".";
      const double yaw = getOrDeclare<double>(prefix + "yaw", 0.0);
      const double delay = getOrDeclare<double>(prefix + "time_from_start", 0.0);
      const auto xs = getOrDeclare<std::vector<double>>(prefix + "x", {});
      const auto ys = getOrDeclare<std::vector<double>>(prefix + "y", {});
      const auto zs = getOrDeclare<std::vector<double>>(prefix + "z", {});
      if (xs.empty() || xs.size() != ys.size() || xs.size() != zs.size()) {
        RCLCPP_ERROR(get_logger(), "Invalid coordinates for segment %d", i);
        continue;
      }
      nav_msgs::msg::Path path;
      path.header.stamp = triggered_time_ + rclcpp::Duration::from_seconds(delay);
      path.header.frame_id = frame_id_;
      for (std::size_t k = 0; k < xs.size(); ++k) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        const double angle = base_yaw + yaw;
        pose.pose.position.x = std::cos(angle) * xs[k] - std::sin(angle) * ys[k] + odom_.pose.pose.position.x;
        pose.pose.position.y = std::sin(angle) * xs[k] + std::cos(angle) * ys[k] + odom_.pose.pose.position.y;
        pose.pose.position.z = zs[k] + odom_.pose.pose.position.z;
        pose.pose.orientation = waypoint_patterns::yawQuaternion(angle);
        path.poses.push_back(pose);
      }
      segments_.push_back(path);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoint segments", segments_.size());
  }

  /** 20 ms 定时检查队首时间戳，到时后发布该段并从队列移除。 */
  void publishDueSegment()
  {
    if (segments_.empty()) {return;}
    if (now() >= rclcpp::Time(segments_.front().header.stamp)) {
      auto path = segments_.front();
      segments_.pop_front();
      publishPath(path);
    }
  }

  std::string waypoint_type_, frame_id_;
  bool have_odom_{false};
  rclcpp::Time triggered_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path pending_manual_;
  std::deque<nav_msgs::msg::Path> segments_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, trigger_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr visualization_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/** 航点生成器入口。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointGenerator>());
  rclcpp::shutdown();
  return 0;
}
