# AlgoVauit · 路径规划算法

本分支保存 `~/ws11` 的完整快照，重点整理基于 ROS 2 Humble 的二维路径规划算法、RViz2 可视化与性能对比。

分支导航：[main](https://github.com/Jon-wang6/AlgoVauit/tree/main) · **path-planning**

## 获取本分支

```bash
git clone --branch path-planning --single-branch https://github.com/Jon-wang6/AlgoVauit.git ws11
cd ws11
```

仓库仍保留早期三维路径规划示例代码，仅供参考，本 README 不再展开介绍。

## 二维算法

核心包 `grid_path_searcher_2d` 在同一张随机二维栅格地图上运行并对比以下算法：

| 算法 | 主要特点 |
| --- | --- |
| Dijkstra | 无启发式的最短路基准算法。 |
| A* | 使用启发函数加速栅格搜索。 |
| TimeBreak A* | 受时间预算约束的 A* 版本。 |
| JPS | 通过跳点和强迫邻居减少扩展节点。 |
| PRM | 先随机采样构建路线图，再执行图搜索。 |
| RRT | 通过随机树快速探索可行空间。 |
| RRT* | 通过择父与重连逐步优化路径。 |
| Anytime RRT* | 在给定时间内持续改进已有解。 |
| Informed RRT* | 获得初始解后在椭圆区域内集中采样。 |
| Kinodynamic RRT* | 在运动学约束下扩展并优化轨迹。 |

## 核心文件

| 文件 | 功能 |
| --- | --- |
| `src/grid_path_searcher_2d/include/grid_path_searcher_2d/grid_searcher_2d.hpp` | 声明二维地图索引、搜索结果和算法接口。 |
| `src/grid_path_searcher_2d/src/grid_map_2d.cpp` | 实现坐标转换、占据判断、路径回溯与碰撞检测。 |
| `src/grid_path_searcher_2d/src/grid_search_astar.cpp` | 实现 Dijkstra、A* 与 TimeBreak A*。 |
| `src/grid_path_searcher_2d/src/grid_search_jps.cpp` | 实现 JPS 邻居判断、跳跃和搜索主循环。 |
| `src/grid_path_searcher_2d/src/grid_search_prm.cpp` | 实现 PRM 采样、路线图构建和图上搜索。 |
| `src/grid_path_searcher_2d/src/grid_search_rrt.cpp` | 实现基础 RRT。 |
| `src/grid_path_searcher_2d/src/grid_search_rrt_star.cpp` | 实现 RRT*。 |
| `src/grid_path_searcher_2d/src/grid_search_anytime_rrt_star.cpp` | 实现 Anytime RRT*。 |
| `src/grid_path_searcher_2d/src/grid_search_informed_rrt_star.cpp` | 实现 Informed RRT*。 |
| `src/grid_path_searcher_2d/src/grid_search_kinodynamic_rrt_star.cpp` | 实现 Kinodynamic RRT*。 |
| `src/grid_path_searcher_2d/src/random_map_node.cpp` | 生成带随机障碍和保底通道的二维 OccupancyGrid。 |
| `src/grid_path_searcher_2d/src/demo_node.cpp` | 接收地图和目标点，运行算法并发布路径与搜索结果。 |
| `src/grid_path_searcher_2d/src/rrt_star_panel.cpp` | 提供 RViz2 实时参数面板。 |
| `src/grid_path_searcher_2d/launch/demo_launch.py` | 启动二维地图、规划节点、目标点节点与 RViz2。 |
| `src/grid_path_searcher_2d/config/demo_2d.rviz` | 配置地图、搜索区域和各算法路径的显示效果。 |

推荐按“公共地图工具 → A*/Dijkstra → JPS → PRM → RRT 系列 → 演示节点”的顺序阅读源码。

## 运行逻辑

```text
random_map_node
  └─ 随机二维 OccupancyGrid
       └─ demo_node
            ├─ Dijkstra / A* / TimeBreak A*
            ├─ JPS / PRM
            ├─ RRT / RRT*
            ├─ Anytime RRT* / Informed RRT*
            └─ Kinodynamic RRT*
                 └─ 路径、搜索范围、耗时结果 ──> RViz2

RViz2 目标点
  └─ waypoint_generator
       └─ demo_node 重新规划
```

演示默认从 `(-8, -8)` 规划到 `(8, 8)`。设置新目标后，所有算法会在相同地图和起终点条件下重新运行，方便观察路径形态、搜索范围和耗时差异。

## 编译与启动

```bash
cd ~/ws11
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch grid_path_searcher_2d demo_launch.py
```

RViz2 打开后可按 `G` 使用目标点工具；二维规划只读取目标的 X、Y 坐标。

## 可复现实验

默认随机地图可以改用固定种子，便于在相同障碍环境下重复对比：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py map_seed:=11
```

使用 `map_seed:=-1` 时，每次启动都会生成新的随机地图。TimeBreak A* 默认时间预算为 2 ms；PRM 与 RRT 系列的采样数量、迭代次数、步长、目标偏置和随机种子可通过启动参数或 RViz2 面板调整。

## RViz2 参数面板

参数面板支持在运行时调整规划参数并重新执行算法，适合观察参数变化对成功率、路径质量、访问节点数量和运行耗时的影响。终端与 RViz2 会同步展示各算法结果，便于横向比较。

## 参考资料

- [ROS 2 二维路径规划算法学习与实现](https://www.yuque.com/g/jonwang-pfbbk/gd0so3/aisohqo8yfbk70np/collaborator/join?token=QXzK0gTytpligebR&source=doc_collaborator#)
