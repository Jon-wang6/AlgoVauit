"""二维随机地图与路径规划演示的总启动文件。

文件职责：使用相同地图和起终点参数启动 random_map_2d、demo_node_2d、
waypoint_generator、Goal3DTool 和可选 RViz2，形成地图生成、自动规划和交互重规划链路。

包含的类：无。
包含的函数：generate_launch_description()，声明启动参数并返回全部 Node Action。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """创建二维地图、规划器、航点生成器和 RViz2 四个运行组件。"""
    rviz_config = (
        get_package_share_directory("grid_path_searcher_2d")
        + "/config/demo_2d.rviz"
    )
    common_parameters = {
        "map.resolution": 0.2,
        "map.width": 20.0,
        "map.height": 20.0,
        "planning.start_x": -8.0,
        "planning.start_y": -8.0,
        "planning.goal_x": 8.0,
        "planning.goal_y": 8.0,
    }
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_demo", default_value="true"),
        DeclareLaunchArgument(
            "map_seed",
            default_value="-1",
            description="地图随机种子；-1 表示每次启动生成新地图，非负值用于复现",
        ),
        Node(
            package="grid_path_searcher_2d",
            executable="demo_node_2d",
            name="demo_node_2d",
            output="screen",
            parameters=[common_parameters, {
                "planning.auto_demo": LaunchConfiguration("auto_demo"),
                "map.occupied_threshold": 50,
                "map.allow_unknown": False,
                "astar.heuristic_weight": 1.0,
                "timed_astar.heuristic_weight": 1.0,
                "jps.heuristic_weight": 1.0,
                "prm.sample_count": 500,
                "prm.k_neighbors": 15,
                "prm.seed": 23,
                "rrt.max_iterations": 8000,
                "rrt.step_size": 5.0,
                "rrt.goal_bias": 0.12,
                "rrt.seed": 31,
                "rrt_star.max_iterations": 5000,
                "rrt_star.step_size": 5.0,
                "rrt_star.rewire_radius": 12.0,
                "rrt_star.goal_bias": 0.12,
                "rrt_star.seed": 37,
                "kinodynamic_rrt_star.max_iterations": 6000,
                "kinodynamic_rrt_star.time_step": 0.8,
                "kinodynamic_rrt_star.max_speed": 6.0,
                "kinodynamic_rrt_star.max_acceleration": 4.0,
                "kinodynamic_rrt_star.rewire_radius": 10.0,
                "kinodynamic_rrt_star.goal_bias": 0.15,
                "kinodynamic_rrt_star.seed": 41,
                "anytime_rrt_star.max_iterations": 12000,
                "anytime_rrt_star.time_budget_ms": 200.0,
                "anytime_rrt_star.step_size": 5.0,
                "anytime_rrt_star.rewire_radius": 12.0,
                "anytime_rrt_star.goal_bias": 0.12,
                "anytime_rrt_star.seed": 43,
                "informed_rrt_star.max_iterations": 7000,
                "informed_rrt_star.step_size": 5.0,
                "informed_rrt_star.rewire_radius": 12.0,
                "informed_rrt_star.goal_bias": 0.12,
                "informed_rrt_star.seed": 47,
                "timed_astar.time_limit_ms": 2.0,
            }],
        ),
        Node(
            package="grid_path_searcher_2d",
            executable="random_map_2d",
            name="random_map_2d",
            output="screen",
            parameters=[common_parameters, {
                "map.obstacle_count": 55,
                "map.seed": ParameterValue(
                    LaunchConfiguration("map_seed"), value_type=int
                ),
            }],
        ),
        Node(
            package="waypoint_generator",
            executable="waypoint_generator",
            name="waypoint_generator",
            output="screen",
            parameters=[{"waypoint_type": "manual-lonely-waypoint", "frame_id": "map"}],
            # 与三维演示使用同一个 /goal，二维规划器只读取目标的 X/Y。
            remappings=[("~/goal", "/goal")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_2d",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
