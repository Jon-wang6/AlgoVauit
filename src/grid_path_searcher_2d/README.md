# grid_path_searcher_2d

> **文件说明**：本文件是二维包的使用说明，不定义程序类或函数；内容包括包结构、
> 节点与话题、算法流程、编译启动方法和 RViz 操作。

这是参照 `grid_path_searcher` 编写的独立二维 ROS 2 Humble 功能包。每个占据
栅格都以独立的方形像素显示；地面采用与地图分辨率一致的 `0.2 m` 小格，
形成 `100 × 100` 网格。它不会修改原来的三维包或系统 ROS 环境。

## 文件和类

| 文件 | 类/函数与职责 |
|---|---|
| `include/grid_path_searcher_2d/grid_searcher_2d.hpp` | 声明数据结构及每种算法名称明确的公开入口。 |
| `src/grid_map_2d.cpp` | 地图判断、坐标转换、路径回溯和直线碰撞检测。 |
| `src/grid_search_astar.cpp` | Dijkstra、A* 与 TimeBreak A*，可直接对比三者区别。 |
| `src/grid_search_jps.cpp` | 强迫邻居、跳跃函数和 JPS 主循环。 |
| `src/grid_search_prm.cpp` | PRM 采样、连边和路线图 Dijkstra。 |
| `src/grid_search_rrt.cpp` | RRT 的 Sample、Nearest、Steer 和路径回溯。 |
| `src/grid_search_rrt_star.cpp` | RRT* 的择父、重连和子树代价更新。 |
| `src/grid_search_kinodynamic_rrt_star.cpp` | 带位置、速度和加速度约束的 Kinodynamic-RRT*。 |
| `src/grid_search_anytime_rrt_star.cpp` | 在时间预算内持续优化并剪枝的 Anytime-RRT*。 |
| `src/grid_search_informed_rrt_star.cpp` | 首解后在启发椭圆内采样的 Informed RRT*。 |
| `src/random_map_node.cpp` | `RandomMapNode` 生成随机二维占据地图。 |
| `src/demo_node.cpp` | `PlannerDemoNode` 调用算法并发布 RViz 数据。 |
| `launch/demo_launch.py` | `generate_launch_description()` 启动完整演示。 |
| `config/demo_2d.rviz` | 显示地图、访问区域和两条路径。 |
| `src/rrt_star_panel.cpp` | RViz 内实时调整地图和全部可调算法参数。 |

## 源码阅读顺序

代码按学习过程组织，而不是只追求行数最少。建议依次阅读：

1. 在头文件查看 `GridIndex`、`SearchResult` 和各算法的独立入口；
2. 阅读 `setMap()`、`traversable()`，理解地图如何判断自由格和障碍格；
3. 阅读 `searchDijkstra()`、`searchAStar()` 及共用的
   `searchWithPriorityQueue()`，重点理解开放队列、松弛和父节点回溯；
4. 阅读 `searchTimedAStar()`，观察时间检查在普通 A* 的哪一步发生；
5. 阅读 `hasForcedNeighbor()`、`jump()`、`searchJps()`，理解 JPS 如何跳格；
6. 阅读 `searchPrm()` 的采样、连边、视线检测和图上 Dijkstra；
7. 最后对比 `searchRrt()` 与 `searchRrtStar()` 的采样、扩展、择父和重连。

A* 和 Dijkstra 虽然共用核心循环，但对外不再使用难以理解的布尔参数。调用处
直接写 `searchAStar(start, goal)` 或 `searchDijkstra(start, goal)`；PRM 和
RRT* 的邻居记录也使用 `node_index`、`distance` 等命名字段，不使用含义不明的
`.first`、`.second`。

## 运行逻辑

```text
random_map_2d --OccupancyGrid--> demo_node_2d
                                  ├─ A* 搜索 → 红色路径 + 橙色访问节点
                                  ├─ TimeBreak A* → 粉红路径 + 紫色访问节点
                                  ├─ Dijkstra → 蓝色对照路径
                                  ├─ JPS → 绿色路径 + 亮青色跳点
                                  ├─ PRM → 灰色路线图 + 洋红色路径
                                  ├─ RRT → 棕色搜索树 + 橙色路径
                                  ├─ RRT* → 蓝绿色重连树 + 亮黄色路径
                                  ├─ Kinodynamic-RRT* → 动力学树 + 青色路径
                                  ├─ Anytime-RRT* → 持续改进树 + 粉色路径
                                  └─ Informed RRT* → 椭圆采样树 + 亮绿色路径

RViz Goal3DTool → /goal → waypoint_generator
                                  → /waypoint_generator/waypoints
                                  → demo_node_2d 重新规划
```

## 编译和运行

```bash
cd ~/ws11
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch grid_path_searcher_2d demo_launch.py
```

启动后会自动从 `(-8,-8)` 规划到 `(8,8)`。在 RViz 顶部选择 **3D Nav Goal**
（快捷键 `G`），像三维演示一样依次设置位置、方向和高度即可指定新目标；二维
规划器只使用目标的 X/Y，忽略 Z 高度。绿色像素是起点，紫色像素是当前终点。

所有方块（障碍、起点、终点和 JPS 跳点）的平面尺寸都保持为普通栅格大小，
不会再用放大方块遮挡周围单元。

二维地图默认使用 `map_seed:=-1`，因此每次重新启动 Launch 都会生成一张新的
随机障碍地图。需要复现某次固定布局时，可以指定非负种子：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py map_seed:=11
```

注意，同一次运行中节点每两秒重新发布的是同一张地图；只有重新启动 Launch
才会生成新地图，避免规划过程中地图突然改变。

TimeBreak A* 紧跟在普通 A* 后运行，默认时间上限为 2 ms。搜索每轮扩展前检查
单调时钟，超过时间立即退出并输出 timeout；在预算内到达终点时则发布与普通
A* 同样的最短路径。

PRM 默认固定随机种子采样 500 个自由节点，每个节点尝试连接 15 个最近的
无碰撞邻居，然后在路线图上用 Dijkstra 求最短路径。可在
`launch/demo_launch.py` 中调整 `prm.sample_count`、`prm.k_neighbors` 和
`prm.seed`。

RRT 默认最多迭代 8000 次，每次最多扩展 5 个栅格（1 米），并以 0.12 的概率
直接采样终点。`rrt.seed=31` 保证默认演示可重复；这些参数都可在 launch 文件
中调整。

RRT* 默认迭代 5000 次、扩展步长 5 格、重连半径 12 格、目标偏置 0.12。
每个新节点会先在邻域中选择累计代价最低的父节点，再尝试重连附近节点；算法
用完迭代预算后才选择最低代价的终点连接，因此通常比普通 RRT 路径更短。

Kinodynamic-RRT* 的树节点包含 `(x,y,vx,vy)`，控制输入是受限的 `(ax,ay)`；
它用双积分器方程推进一个时间步，只接受整段无碰撞且动力学可达的择父和重连边。

Anytime-RRT* 得到首条路径后不会立即结束，而是在时间/迭代预算内继续重连优化，
并用当前最优路径代价做分支限界剪枝，最终返回截止时保存的最好路径。

Informed RRT* 首解前全地图采样；首解后只在以起点和终点为焦点、当前最优路径
长度决定长轴的椭圆内采样，把后续计算集中到可能改善路径的区域。

RViz 左侧的 **全部算法参数** 面板可实时修改地图通行规则、A*/TimeBreak A*/JPS
启发权重、TimeBreak 时间上限，以及 PRM、RRT、RRT*、Kinodynamic-RRT*、
Anytime-RRT*、Informed RRT* 的全部可调参数。面板可滚动；停止输入 250 ms 后
参数自动发送，`demo_node_2d` 会立即使用当前终点重新规划。Dijkstra 严格按累计
代价 `g` 排序，本身没有启发权重或随机采样参数，所以面板只显示说明。

启发权重为 `1` 时是标准 A*/JPS；设为 `0` 时按 Dijkstra 式优先级搜索；大于
`1` 时会更偏向终点、通常扩展更少，但不再保证得到最短路径。

每次规划都会输出全部十种算法的计算耗时，单位为毫秒。
PRM 的计时包含随机采样、路线图构建、视线碰撞检测和图搜索，不包含 RViz
消息生成与发布；其他算法同样只统计搜索调用，因此可以直接比较计算开销。
