"""二维 OccupancyGrid 兼容规划链路启动文件。

文件职责：声明地图、里程计和 RViz 开关参数，启动二维 A* 节点、航点生成器
与可选 RViz2，并完成私有话题到公共话题的映射。

包含的类：无。
包含的函数：generate_launch_description()，创建并返回完整启动描述。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """声明可覆盖的话题参数，并创建二维规划链路中的三个节点。"""
    rviz_config = get_package_share_directory("rviz_plugins") + "/config/ros2_demo.rviz"
    return LaunchDescription([
        DeclareLaunchArgument("map_topic", default_value="/grid_map_global"),
        DeclareLaunchArgument("odom_topic", default_value="/laser_localization"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 二维 A* 节点；私有 waypoints 被映射到航点生成器的输出。
        Node(
            package="grid_path_searcher", executable="occupancy_demo_node", name="demo_node",
            output="screen",
            parameters=[{
                "map_topic": LaunchConfiguration("map_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
            }],
            remappings=[("~/waypoints", "/waypoint_generator/waypoints")],
        ),
        # 将 /goal 转换成 /waypoint_generator/waypoints。
        Node(
            package="waypoint_generator", executable="waypoint_generator",
            name="waypoint_generator", output="screen",
            parameters=[{"waypoint_type": "manual-lonely-waypoint"}],
            remappings=[("~/goal", "/goal")],
        ),
        # 加载二维地图与路径显示配置。
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
