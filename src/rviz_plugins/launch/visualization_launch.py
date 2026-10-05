"""单独启动二维可视化界面的 launch 文件。

文件职责：从已安装的 rviz_plugins 包中解析 ros2_demo.rviz 路径，然后启动
一个加载该配置的 RViz2 进程；适用于规划节点已经独立运行的情况。

包含的类：无。
包含的函数：generate_launch_description()，创建 RViz2 Node Action。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """从安装后的 rviz_plugins 共享目录解析配置文件并创建 RViz2 节点。"""
    config = get_package_share_directory("rviz_plugins") + "/config/ros2_demo.rviz"
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", config],
        )
    ])
