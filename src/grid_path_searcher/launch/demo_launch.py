"""三维路径规划教程的总启动文件。

文件职责：以一致的地图参数启动 random_complex、demo_node、
waypoint_generator 和可选 RViz2，形成随机地图到交互重规划的完整链路。

包含的类：无。
包含的函数：generate_launch_description()，声明启动参数并返回所有 ROS 2
节点组成的 LaunchDescription。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """组装三维演示的全部 ROS 2 Action，并把相同地图参数传给生成器和规划器。"""
    # RViz 配置由 rviz_plugins 包安装，运行时通过 ament 索引获取绝对路径。
    rviz_config = get_package_share_directory("rviz_plugins") + "/config/tutorial_3d.rviz"
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_demo", default_value="true"),
        # 三维搜索节点：订阅随机点云和航点，发布地图、路径及搜索节点 Marker。
        Node(
            package="grid_path_searcher", executable="demo_node", name="demo_node",
            output="screen",
            parameters=[{
                "map.resolution": 0.2,
                "map.x_size": 20.0,
                "map.y_size": 20.0,
                "map.z_size": 4.0,
                "planning.start_x": 0.0,
                "planning.start_y": 0.0,
                "planning.start_z": 1.0,
                "planning.auto_demo": LaunchConfiguration("auto_demo"),
            }],
        ),
        # 随机地图节点：固定 seed=7，保证每次演示的障碍布局可重复。
        Node(
            package="grid_path_searcher", executable="random_complex",
            name="random_complex", output="screen",
            parameters=[{
                "map.resolution": 0.2,
                "map.x_size": 20.0,
                "map.y_size": 20.0,
                "map.z_size": 4.0,
                "map.obstacle_count": 180,
                "map.ring_count": 32,
                "map.seed": 7,
                "init_state_x": 0.0,
                "init_state_y": 0.0,
            }],
        ),
        # 航点节点：把 RViz /goal 转换为规划器使用的 Path。
        Node(
            package="waypoint_generator", executable="waypoint_generator",
            name="waypoint_generator", output="screen",
            parameters=[{"waypoint_type": "manual-lonely-waypoint"}],
            remappings=[("~/goal", "/goal")],
        ),
        # use_rviz=false 时只启动计算节点，便于在另一个终端单独启动 RViz。
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
