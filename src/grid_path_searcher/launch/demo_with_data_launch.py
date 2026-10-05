"""带教程 rosbag2 地图数据的二维兼容演示启动文件。

文件职责：包含 occupancy_demo_launch.py，并在节点就绪后循环播放转换自
ROS 1 教程的 OccupancyGrid 数据，同时加载匹配的 QoS 覆盖配置。

包含的类：无。
包含的函数：generate_launch_description()，组合子 launch、延时器与
ros2 bag play 进程。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """先包含二维节点 launch，延时两秒后以 10 倍速循环播放地图数据。"""
    package_share = get_package_share_directory("grid_path_searcher")
    demo_launch = package_share + "/launch/occupancy_demo_launch.py"
    bag_path = package_share + "/data/tutorial_demo"
    qos_path = package_share + "/data/qos_overrides.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 复用 occupancy_demo_launch，避免重复定义规划器、航点与 RViz 节点。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
        ),
        # 等待订阅器建立后再播放，QoS 覆盖保证持久性策略与地图订阅兼容。
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "bag", "play", bag_path,
                        "--qos-profile-overrides-path", qos_path,
                        "--loop", "--rate", "10.0",
                    ],
                    output="screen",
                )
            ],
        ),
    ])
