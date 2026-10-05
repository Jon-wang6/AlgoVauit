# AlgoVauit · 路径规划算法

本分支保存 `~/ws11` 的完整快照，是基于 ROS 2 Humble 的二维/三维路径规划、RViz2 可视化与算法性能对比工作空间。

分支导航：[main](https://github.com/Jon-wang6/AlgoVauit/tree/main) · **path-planning**

## 获取本分支

```bash
git clone --branch path-planning --single-branch https://github.com/Jon-wang6/AlgoVauit.git ws11
cd ws11
```

> **文件说明**：本文档是工作空间总览，不定义程序类或函数；内容包括全部文件职责、
> 三维/二维运行链路、各节点输入输出、构建启动命令、RViz 操作方法和 VS Code 配置说明。

本工作空间把教程中的 `grid_path_searcher`、`waypoint_generator` 和 `rviz_plugins` 适配到了 Ubuntu 22.04 + ROS 2 Humble。默认演示使用三维点云地图，同时运行 JPS 和 A*；另保留二维 `OccupancyGrid` 兼容入口。

工作空间另外包含独立的 `grid_path_searcher_2d` 包：它生成随机二维栅格地图，
对比 A*、TimeBreak A*、Dijkstra、JPS、PRM、RRT、RRT*、Anytime RRT*、
Informed RRT* 与 Kinodynamic RRT*，并在 RViz2 中显示搜索范围、路径和实时参数面板。

## 目录与文件职责

### grid_path_searcher

| 文件 | 功能 |
|---|---|
| `src/random_complex_node.cpp` | 生成随机柱体和三维圆环，发布 `/random_complex/global_map` 点云。 |
| `src/demo_node.cpp` | 接收三维点云和航点，建立占据栅格，调用 JPS/A*，发布 RViz Marker。 |
| `include/graph_searcher.hpp` | 声明栅格节点、JPS 邻居表和三维搜索器接口。 |
| `src/graph_searcher.cpp` | 实现坐标转换、占据检查、A*、JPS、跳点递归和路径回溯。 |
| `src/occupancy_demo_node.cpp` | 在二维 `OccupancyGrid` 上执行八邻域 A*，兼容旧教程数据。 |
| `launch/demo_launch.py` | 启动默认三维演示的全部节点。 |
| `launch/occupancy_demo_launch.py` | 启动二维规划器、航点生成器和二维 RViz。 |
| `launch/demo_with_data_launch.py` | 在二维启动文件基础上自动循环播放教程 rosbag2。 |
| `data/tutorial_demo/` | 转换后的二维教程地图包；`.db3` 是二进制消息数据库，`metadata.yaml` 是索引。 |

### waypoint_generator

| 文件 | 功能 |
|---|---|
| `src/waypoint_generator.cpp` | 接收 `/goal`，根据模式生成手动或预设航点并发布 `nav_msgs/Path`。 |
| `include/sample_waypoints.hpp` | 提供单点折线、圆形和三维八字形的预设控制点。 |

### grid_path_searcher_2d

| 文件 | 功能 |
|---|---|
| `include/grid_path_searcher_2d/grid_searcher_2d.hpp` | 声明二维索引、搜索结果与搜索器接口。 |
| `src/grid_map_2d.cpp` | 地图、坐标、路径回溯与直线碰撞检测公共工具。 |
| `src/grid_search_astar.cpp` | Dijkstra、A* 与 TimeBreak A*。 |
| `src/grid_search_jps.cpp` | JPS 强迫邻居、跳跃和搜索主循环。 |
| `src/grid_search_prm.cpp` | PRM 随机采样、路线图与图上 Dijkstra。 |
| `src/grid_search_rrt.cpp` | RRT 随机树扩展。 |
| `src/grid_search_rrt_star.cpp` | RRT* 择父、重连与路径优化。 |
| `src/random_map_node.cpp` | 生成带随机障碍和保底通道的二维 OccupancyGrid。 |
| `src/demo_node.cpp` | 接入地图和航点并发布访问节点与两类路径。 |
| `launch/demo_launch.py` | 启动二维完整演示。 |
| `config/demo_2d.rviz` | 二维地图、访问区域和路径显示配置。 |

### rviz_plugins

| 文件 | 功能 |
|---|---|
| `include/pose_tool.hpp` | 声明 RViz 鼠标三维姿态交互基类。 |
| `src/pose_tool.cpp` | 实现位置、朝向和高度三个鼠标交互阶段。 |
| `include/goal_tool.hpp` | 声明发布 ROS 2 目标点的 Goal3DTool。 |
| `src/goal_tool.cpp` | 把交互结果封装成 `PoseStamped` 并发布 `/goal`。 |
| `plugin_description.xml` | 把 Goal3DTool 注册给 RViz2 的 pluginlib。 |
| `config/tutorial_3d.rviz` | 三维点云、JPS/A* 路径和搜索节点显示配置。 |
| `config/ros2_demo.rviz` | 二维地图和全局路径显示配置。 |

`.vscode/` 保存 IntelliSense、编译数据库和任务配置。VS Code 的工作区 JSON 文件支持 JSON with Comments，因此文件头使用 `//` 注释介绍用途与内容，不影响 VS Code 解析。

## 默认三维演示的完整运行逻辑

```text
random_complex
  │ 生成柱体/圆环点云
  ▼
/random_complex/global_map (PointCloud2)
  │
  ▼
demo_node ── 建立三维占据栅格 ── JPS + A*
  │                              │
  │                              ├─ /demo_node/grid_path_vis
  │                              ├─ /demo_node/closed_nodes_vis
  │                              └─ /demo_node/debug_nodes_vis
  └─ /demo_node/grid_map_vis

RViz Goal3DTool
  │ 发布 PoseStamped
  ▼
/goal
  ▼
waypoint_generator
  │ 转成 nav_msgs/Path
  ▼
/waypoint_generator/waypoints
  ▼
demo_node 重新规划
```

具体步骤如下：

1. `demo_launch.py` 启动 `random_complex`、`demo_node`、`waypoint_generator` 和 RViz2。
2. `random_complex` 使用固定随机种子生成可重复的三维障碍点云，并以 Transient Local QoS 发布。
3. `demo_node` 收到第一帧点云后，把每个 XYZ 点写入三维占据数组；如果设置了 margin，还会对障碍进行膨胀。
4. `demo_node` 默认以 `(0,0,1)` 为起点，以 `(7,7,1)` 为目标先运行 JPS、再运行普通 A*。
5. 两种算法都使用 `f=g+h`：`g` 是累计路径代价，`h` 是三维对角距离启发项；JPS 通过自然邻居、强迫邻居和递归跳跃减少扩展节点。
6. 搜索成功后沿每个节点的 `cameFrom` 指针从目标回溯到起点，再翻转成正向路径。
7. 地图、路径和扩展节点转换为 PointCloud2/Marker，由 RViz2 以不同颜色显示。
8. 用户使用 Goal3DTool 点击新位置时，插件发布 `/goal`；`waypoint_generator` 将其封装为 Path；`demo_node` 收到后清理上一轮状态并重新运行 JPS 与 A*。

## 二维兼容模式的运行逻辑

```text
rosbag2 或外部地图节点
  └─ /grid_map_global (OccupancyGrid)
       └─ occupancy_demo_node

/laser_localization (Odometry) ─┐
/waypoint_generator/waypoints ──┼─> 八邻域 A* ─> /global_path
                                └──────────────> /demo_node/grid_path_vis
```

二维规划器将世界坐标变换到地图局部栅格坐标，使用直移代价 10、斜移代价 14 的八邻域 A*，最后把栅格中心重新转换到世界坐标并发布路径。

## 编译

```bash
cd /home/jon/ws11
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

## 启动

三维教程演示：

```bash
ros2 launch grid_path_searcher demo_launch.py
```

不自动打开 RViz2：

```bash
ros2 launch grid_path_searcher demo_launch.py use_rviz:=false
```

二维教程数据演示：

```bash
ros2 launch grid_path_searcher demo_with_data_launch.py
```

## Goal3DTool 操作

1. 左键按下确定 XY 位置。
2. 保持左键拖动确定偏航角。
3. 保持左键的同时按住右键并上下拖动调整高度。
4. 松开左键发布 `/goal`。

三维演示使用 `world` 固定坐标系。

## VS Code

- `Ctrl+Shift+B`：编译整个工作空间。
- `Terminal → Run Task → ROS 2: launch 3D demo`：启动三维演示。
- IntelliSense 仍显示旧缓存时执行 `Ctrl+Shift+P → Developer: Reload Window`。
