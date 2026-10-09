
> 学习环境：Ubuntu 22.04、ROS 2 Humble  
工作空间：`~/ws11`  
功能包：`grid_path_searcher_2d`  
编程语言：C++17  
可视化工具：RViz2  
地图类型：二维占据栅格地图 `nav_msgs/msg/OccupancyGrid`
>

本文记录二维路径规划的学习过程，以及在 ROS 2 Humble 中实现和可视化不同路径规划算法的方法。

当前程序已经实现：

+ Dijkstra
+ A*
+ TimeBreak A*
+ JPS
+ PRM
+ RRT
+ RRT*
+ Kinodynamic-RRT*
+ Anytime-RRT*
+ Informed RRT*

DFS、BFS、全局规划器、局部规划器、Nav2、OMPL、MoveIt 2 等内容目前以原理学习为主。

---

# 一、学习目标
本项目的主要学习目标如下：

1. 理解二维占据栅格地图的数据结构。
2. 理解世界坐标和栅格坐标之间的转换。
3. 理解队列、优先队列、父节点表等数据结构。
4. 掌握 Dijkstra 和 A* 的搜索过程。
5. 理解 JPS 如何通过跳点减少搜索节点。
6. 理解 PRM 的采样、连边和视线碰撞检测。
7. 理解 RRT 如何通过随机采样生成搜索树。
8. 理解 RRT* 的选择父节点和重新连接过程。
9. 了解 Kinodynamic-RRT*、Anytime-RRT* 和 Informed RRT*。
10. 使用 RViz2 显示地图、搜索过程、路径和算法参数。
11. 比较不同算法的计算时间和路径效果。

---

# 二、路径规划基础
## 2.1 什么是路径规划
路径规划就是：

> 在地图中，从起点找到一条能够避开障碍物并到达终点的路线。
>

路径规划至少需要以下内容：

+ 一张地图；
+ 一个起点；
+ 一个终点；
+ 障碍物信息；
+ 路径搜索算法；
+ 碰撞检测方法。

本项目使用的是二维占据栅格地图。

---

## 2.2 二维占据栅格地图
二维占据栅格地图将整个平面分成许多大小相同的小格子。

本项目默认参数为：

| 参数 | 数值 | 说明 |
| --- | ---: | --- |
| 地图宽度 | 20 m | X 方向长度 |
| 地图高度 | 20 m | Y 方向长度 |
| 地图分辨率 | 0.2 m | 每个格子的边长 |
| 横向格子数 | 100 | `20 ÷ 0.2` |
| 纵向格子数 | 100 | `20 ÷ 0.2` |
| 总格子数 | 10000 | `100 × 100` |


地图消息类型为：

```cpp
nav_msgs::msg::OccupancyGrid
```

每个格子通常有三种状态：

| 数值 | 含义 |
| ---: | --- |
| `0` | 可以通行 |
| `100` | 被障碍物占据 |
| `-1` | 未知区域 |


程序默认的占据阈值为：

```cpp
map.occupied_threshold = 50
```

当一个格子的数值大于或等于 50 时，程序会把它当成障碍物。

---

## 2.3 世界坐标与栅格坐标
RViz2 中使用的是世界坐标，例如：

```latex
起点：(-8.0, -8.0)
终点：( 8.0,  8.0)
```

路径规划算法内部主要使用整数栅格坐标，例如：

```latex
起点格子：(10, 10)
终点格子：(90, 90)
```

世界坐标转换为栅格坐标的基本公式为：

```latex
grid_x = (world_x - origin_x) / resolution
grid_y = (world_y - origin_y) / resolution
```

栅格坐标转换回世界坐标时，程序使用格子中心：

```latex
world_x = origin_x + (grid_x + 0.5) × resolution
world_y = origin_y + (grid_y + 0.5) × resolution
```

对应源码中的函数为：

```cpp
bool worldToGrid(
    double world_x,
    double world_y,
    GridIndex & index) const;

geometry_msgs::msg::Point gridToWorld(
    const GridIndex & index) const;
```

---

## 2.4 全局规划与局部规划
路径规划通常可以分为全局规划和局部规划。

### 全局规划
全局规划器根据整张地图计算从起点到终点的完整路径。

常见算法包括：

+ Dijkstra
+ A*
+ JPS
+ PRM
+ RRT
+ RRT*

本项目目前实现的算法主要属于全局路径规划算法。

### 局部规划
局部规划器根据机器人附近的障碍物和当前运动状态，计算机器人短时间内应该如何运动。

常见算法包括：

+ DWA
+ TEB
+ E-Band

可以将两者理解为：

```latex
全局规划器：决定总体应该走哪条路
                   ↓
局部规划器：决定机器人当前应该怎么走
                   ↓
底盘控制器：控制线速度和角速度
```

---

# 三、功能包结构
功能包路径为：

```latex
~/ws11/src/grid_path_searcher_2d
```

主要目录结构如下：

```latex
grid_path_searcher_2d/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml
├── README.md
├── config/
│   └── demo_2d.rviz
├── include/
│   └── grid_path_searcher_2d/
│       ├── grid_searcher_2d.hpp
│       └── rrt_star_panel.hpp
├── launch/
│   └── demo_launch.py
└── src/
    ├── demo_node.cpp
    ├── random_map_node.cpp
    ├── grid_map_2d.cpp
    ├── grid_search_astar.cpp
    ├── grid_search_jps.cpp
    ├── grid_search_prm.cpp
    ├── grid_search_rrt.cpp
    ├── grid_search_rrt_star.cpp
    ├── grid_search_kinodynamic_rrt_star.cpp
    ├── grid_search_anytime_rrt_star.cpp
    ├── grid_search_informed_rrt_star.cpp
    └── rrt_star_panel.cpp
```

---

## 3.1 各文件的作用
| 文件 | 作用 |
| --- | --- |
| `grid_searcher_2d.hpp` | 声明数据结构、类和算法接口 |
| `grid_map_2d.cpp` | 地图判断、坐标转换、路径回溯和直线碰撞检测 |
| `grid_search_astar.cpp` | Dijkstra、A* 和 TimeBreak A* |
| `grid_search_jps.cpp` | JPS、强迫邻居和跳点搜索 |
| `grid_search_prm.cpp` | PRM 采样、连边和图搜索 |
| `grid_search_rrt.cpp` | RRT 随机树 |
| `grid_search_rrt_star.cpp` | RRT* 选择父节点和重新连接 |
| `grid_search_kinodynamic_rrt_star.cpp` | 带速度、加速度约束的 RRT* |
| `grid_search_anytime_rrt_star.cpp` | 在时间预算内持续优化路径 |
| `grid_search_informed_rrt_star.cpp` | 首次找到路径后进行椭圆区域采样 |
| `random_map_node.cpp` | 生成随机二维地图 |
| `demo_node.cpp` | 调用所有算法并发布 RViz2 可视化数据 |
| `rrt_star_panel.cpp` | RViz2 参数调整面板 |
| `demo_launch.py` | 同时启动地图、规划器、航点生成器和 RViz2 |
| `demo_2d.rviz` | RViz2 显示配置 |


---

## 3.2 `.hpp` 和 `.cpp` 的分工
头文件 `.hpp` 主要负责声明：

```cpp
class GridSearcher2D
{
public:
  SearchResult searchDijkstra(
      const GridIndex & start,
      const GridIndex & goal) const;

  SearchResult searchAStar(
      const GridIndex & start,
      const GridIndex & goal,
      double heuristic_weight = 1.0) const;
};
```

源文件 `.cpp` 负责实现：

```cpp
SearchResult GridSearcher2D::searchDijkstra(
    const GridIndex & start,
    const GridIndex & goal) const
{
  return searchWithPriorityQueue(
      start,
      goal,
      PriorityMode::kDijkstra,
      0.0);
}
```

可以理解为：

```latex
.hpp：告诉其他文件“有哪些类和函数可以使用”
.cpp：真正写出这些函数具体怎么运行
```

---

# 四、程序整体运行逻辑
完整的数据流如下：

```latex
random_map_2d
    │
    │ 发布二维占据栅格地图
    ▼
/random_map_2d/global_map
    │
    ▼
demo_node_2d
    │
    ├── Dijkstra
    ├── A*
    ├── TimeBreak A*
    ├── JPS
    ├── PRM
    ├── RRT
    ├── RRT*
    ├── Kinodynamic-RRT*
    ├── Anytime-RRT*
    └── Informed RRT*
    │
    ▼
发布路径、访问节点、跳点、路线图和搜索树
    │
    ▼
RViz2 显示
```

交互设置终点的流程如下：

```latex
RViz2 中使用 3D Nav Goal
    │
    ▼
发布 /goal
    │
    ▼
waypoint_generator
    │
    ▼
发布 /waypoint_generator/waypoints
    │
    ▼
demo_node_2d 接收新终点
    │
    ▼
所有算法重新规划
```

二维规划器只读取目标的 X、Y 坐标，忽略 Z 坐标。

---

# 五、C++数据结构基础
## 5.1 `std::vector` 是什么
`std::vector` 是 C++ 中可以自动改变长度的连续数组。

普通数组的长度通常需要提前确定：

```cpp
int numbers[5];
```

`std::vector` 可以根据需要增加元素：

```cpp
std::vector<int> numbers;

numbers.push_back(10);
numbers.push_back(20);
numbers.push_back(30);
```

此时内容为：

```latex
numbers = [10, 20, 30]
```

读取元素的方法为：

```cpp
int first = numbers[0];
int second = numbers[1];
```

在路径规划中，地图有多少个格子，就可以创建多少个元素：

```cpp
std::vector<double> g_score(
    grid_count,
    std::numeric_limits<double>::infinity());

std::vector<int> parent(grid_count, -1);

std::vector<bool> closed(grid_count, false);
```

---

## 5.2 `g_score`
```cpp
std::vector<double> g_score(
    grid_count,
    std::numeric_limits<double>::infinity());
```

`g_score[i]` 表示：

> 从起点走到第 `i` 个格子的当前最小已知代价。
>

最开始还不知道如何到达其他格子，因此将它们设置成无穷大：

```latex
g_score = [∞, ∞, ∞, ∞, ...]
```

起点到起点的距离为 0：

```cpp
g_score[start_index] = 0.0;
```

之后可能变成：

```latex
g_score = [0, 1, 2, 2.414, 3.414, ...]
```

---

## 5.3 `parent`
```cpp
std::vector<int> parent(grid_count, -1);
```

`parent[i]` 表示：

> 搜索到第 `i` 个格子时，是从哪个格子走过来的。
>

例如：

```latex
parent[终点] = 35
parent[35]   = 26
parent[26]   = 17
parent[17]   = 起点
```

搜索完成后，从终点不断查询父节点：

```latex
终点 → 35 → 26 → 17 → 起点
```

然后将顺序反转：

```latex
起点 → 17 → 26 → 35 → 终点
```

这样就得到了完整路径。

---

## 5.4 `closed`
```cpp
std::vector<bool> closed(grid_count, false);
```

`closed[i]` 表示：

> 第 `i` 个格子是否已经正式处理完成。
>

开始时所有格子都没有处理：

```latex
closed = [false, false, false, ...]
```

取出一个当前代价最小的格子后：

```cpp
closed[current] = true;
```

如果以后队列中又出现相同格子的旧记录：

```cpp
if (closed[current])
{
  continue;
}
```

程序会直接跳过，避免重复处理。

---

## 5.5 队列
队列的特点是：

```latex
先进入队列的元素，先出来
```

类似排队买票：

```latex
队尾进入 → [A] [B] [C] → 队头离开
```

普通队列适合 BFS，因为 BFS 按进入顺序逐层搜索。

```cpp
std::queue<int> open_queue;

open_queue.push(start);
int current = open_queue.front();
open_queue.pop();
```

---

## 5.6 优先队列
优先队列不是按进入顺序取元素，而是根据优先级取元素。

路径规划中希望每次取出“当前最值得处理”的格子：

+ Dijkstra 取 `g` 最小的格子；
+ A* 取 `g+h` 最小的格子。

代码：

```cpp
int current = open_queue.top().index;
open_queue.pop();
```

分成两步理解：

```cpp
open_queue.top()
```

查看优先队列顶部，也就是当前优先级最高的元素。

```cpp
open_queue.top().index
```

从这个元素中读取地图格子编号。

```cpp
open_queue.pop()
```

将刚才读取的元素从队列中删除。

因此这两行的完整含义是：

> 找到待搜索节点中代价最小的节点，记录它的编号，然后把它从待搜索队列中移除。
>

---

# 六、DFS和BFS
> 当前 `grid_path_searcher_2d` 没有单独实现 DFS 和 BFS，本节主要用于理解图搜索基础。
>

## 6.1 DFS
DFS 的中文名称是深度优先搜索。

它的搜索方式类似：

```latex
选择一个方向
    ↓
一直向前搜索
    ↓
走不通时返回
    ↓
尝试其他方向
```

DFS 通常使用栈或者递归。

优点：

+ 实现简单；
+ 占用内存较少；
+ 适合理解回溯。

缺点：

+ 不保证最短路径；
+ 可能先走很长的错误路线；
+ 不适合直接作为栅格最短路径算法。

---

## 6.2 BFS
BFS 的中文名称是广度优先搜索。

它会一层一层地搜索：

```latex
距离起点1步的格子
        ↓
距离起点2步的格子
        ↓
距离起点3步的格子
```

BFS 使用普通队列。

当所有移动代价相同时，BFS 可以找到步数最少的路径。

如果允许斜向移动，并且斜向代价为 `√2`，不同边的代价不再相同，此时应该使用 Dijkstra。

---

# 七、Dijkstra算法
## 7.1 基本思想
Dijkstra 每次选择：

> 从起点到当前节点累计代价 `g` 最小的节点。
>

它不关心终点在哪个方向，只关心当前已经走过的真实距离。

因此它会像水波一样从起点向四周扩散。

---

## 7.2 Dijkstra每一步的逻辑
### 第一步：初始化
```cpp
g_score[start_index] = 0.0;
open_queue.push(start_node);
```

含义：

+ 起点到自己的代价是 0；
+ 把起点放入待搜索队列。

### 第二步：检查队列是否为空
```cpp
while (!open_queue.empty())
```

只要队列中还有可以搜索的格子，就继续执行。

如果队列为空仍未找到终点，说明不存在可行路径。

### 第三步：取出代价最小的格子
```cpp
int current = open_queue.top().index;
open_queue.pop();
```

Dijkstra 的优先队列按照 `g_score` 排序，所以这里取出的是累计代价最小的格子。

### 第四步：跳过旧记录
```cpp
if (closed[current])
{
  continue;
}
```

同一个节点可能被多次放进优先队列。

当发现它已经处理完成时，就跳过这条旧记录。

### 第五步：标记处理完成
```cpp
closed[current] = true;
```

说明当前已经找到了到达这个格子的最小代价。

### 第六步：判断是否到达终点
```cpp
if (current == goal_index)
{
  break;
}
```

如果当前节点就是终点，就可以结束搜索。

### 第七步：检查邻居
```cpp
for (int neighbor : get_neighbors(current))
```

获取当前格子周围所有可以通行的邻居。

二维八邻域包括：

```latex
↖  ↑  ↗
← 当前 →
↙  ↓  ↘
```

### 第八步：计算新路线代价
```cpp
double new_cost =
    g_score[current] +
    movement_cost(current, neighbor);
```

如果直线移动，代价通常为：

```latex
1
```

如果斜向移动，代价通常为：

```latex
√2 ≈ 1.414
```

### 第九步：进行松弛
```cpp
if (new_cost < g_score[neighbor])
{
  g_score[neighbor] = new_cost;
  parent[neighbor] = current;
  open_queue.push(...);
}
```

如果经过当前节点到达邻居更短：

1. 更新邻居的最短距离；
2. 记录邻居来自当前节点；
3. 把邻居重新加入优先队列。

这个过程称为“松弛”。

---

## 7.3 Dijkstra完整流程
```latex
初始化起点
    ↓
起点加入优先队列
    ↓
队列是否为空？——是——→ 搜索失败
    │
    否
    ↓
取出g最小的节点
    ↓
节点是否处理过？——是——→ 跳过
    │
    否
    ↓
标记为已处理
    ↓
是否为终点？——是——→ 回溯路径
    │
    否
    ↓
遍历所有邻居
    ↓
计算经过当前节点的新代价
    ↓
新代价是否更小？
    │
    ├── 是：更新g、parent并加入队列
    └── 否：保持原数据
```

---

## 7.4 Dijkstra特点
优点：

+ 保证找到最短路径；
+ 原理清楚；
+ 不需要启发函数；
+ 适合作为其他算法的对照组。

缺点：

+ 不知道终点方向；
+ 会搜索大量无关区域；
+ 大地图中搜索速度可能较慢。

---

# 八、A*算法
## 8.1 基本思想
A* 在 Dijkstra 的基础上增加了启发函数。

Dijkstra 使用：

```latex
priority = g
```

A* 使用：

```latex
priority = g + h
```

其中：

+ `g`：起点到当前节点的真实代价；
+ `h`：当前节点到终点的估计代价；
+ `f`：节点的总优先级。

---

## 8.2 启发函数
本项目采用适合八邻域的 Octile Distance。

它会同时考虑：

+ 横向移动；
+ 纵向移动；
+ 斜向移动。

标准 A* 的启发权重为：

```latex
heuristic_weight = 1.0
```

计算方式为：

```latex
priority = g + 1.0 × h
```

参数含义如下：

| 启发权重 | 效果 |
| ---: | --- |
| `0` | 接近 Dijkstra |
| `1` | 标准 A* |
| 大于 `1` | 更偏向终点，搜索更快，但不再保证最短 |


---

## 8.3 A*和Dijkstra共用函数
源码中两个算法共用：

```cpp
SearchResult searchWithPriorityQueue(
    const GridIndex & start,
    const GridIndex & goal,
    PriorityMode mode,
    double heuristic_weight) const;
```

Dijkstra 调用：

```cpp
return searchWithPriorityQueue(
    start,
    goal,
    PriorityMode::kDijkstra,
    0.0);
```

A* 调用：

```cpp
return searchWithPriorityQueue(
    start,
    goal,
    PriorityMode::kAStar,
    heuristic_weight);
```

两者共同使用：

+ 地图边界检查；
+ 障碍物检查；
+ 八邻域遍历；
+ `g_score`；
+ `parent`；
+ `closed`；
+ 路径回溯；
+ 松弛操作。

两者的主要区别只有优先级：

```cpp
if (mode == PriorityMode::kDijkstra)
{
  priority = new_cost;
}
else
{
  priority =
      new_cost +
      heuristic_weight * heuristic(neighbor, goal);
}
```

---

## 8.4 A*每一步逻辑
```latex
设置起点g=0
    ↓
计算起点f=g+h
    ↓
起点加入优先队列
    ↓
取出f最小的节点
    ↓
判断是否到达终点
    ↓
遍历可通行邻居
    ↓
计算新的g值
    ↓
如果新的g值更小
    ↓
更新g和parent
    ↓
计算新的f=g+h
    ↓
邻居加入优先队列
```

---

## 8.5 A*特点
优点：

+ 标准条件下可以找到最短路径；
+ 通常比 Dijkstra 搜索的节点少；
+ 目标方向明确；
+ 适合规则栅格地图。

缺点：

+ 地图较大时仍可能访问很多格子；
+ 启发函数设计会影响搜索效果；
+ 动态环境中需要重新规划。

---

# 九、TimeBreak A*
## 9.1 当前程序中的真实含义
当前项目将它命名为 `TimeBreak A*`，但代码实际实现的是：

> 带时间预算限制的 A*。
>

它不是“A_优先级相同时的平局打破策略”，而是在普通 A_ 搜索中增加超时判断。

默认时间限制为：

```latex
2 ms
```

---

## 9.2 搜索过程
整体过程与 A* 相同：

```latex
开始搜索
    ↓
记录开始时间
    ↓
每次准备扩展节点前检查时间
    ↓
是否超过时间限制？
    │
    ├── 是：停止并返回 timed_out=true
    └── 否：继续执行A*
```

核心逻辑可以理解为：

```cpp
const auto start_time = std::chrono::steady_clock::now();

while (!open_queue.empty())
{
  const auto current_time =
      std::chrono::steady_clock::now();

  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
          current_time - start_time).count();

  if (elapsed_ms > time_limit_ms)
  {
    result.timed_out = true;
    return result;
  }

  // 继续普通A*搜索
}
```

---

## 9.3 可调参数
```latex
timed_astar.heuristic_weight
timed_astar.time_limit_ms
```

如果时间预算太小，算法可能在到达终点前停止。

如果时间预算足够，结果通常与普通 A* 相同。

---

# 十、JPS算法
## 10.1 JPS是什么
JPS 全称为 Jump Point Search，即跳点搜索。

普通 A* 会一个格子一个格子地扩展。

JPS 会跳过没有转折意义的普通格子，只把重要节点作为跳点。

例如：

```latex
普通A*：

S → □ → □ → □ → □ → G

JPS：

S ─────────────→ G
```

JPS 不是忽略中间格子的碰撞检测，而是不把每个中间格子都放进开放队列。

---

## 10.2 强迫邻居
强迫邻居是 JPS 的核心概念。

如果障碍物迫使路径必须从某个位置转弯，那么这个位置就可能成为跳点。

示意：

```latex
■ 表示障碍物
○ 表示当前节点
× 表示被迫需要检查的邻居

■ ×
○ →
```

因为障碍物挡住了正常方向，算法必须考虑旁边的特殊方向。

---

## 10.3 跳跃函数
程序中的跳跃函数为：

```cpp
bool jump(
    const GridIndex & current,
    int dx,
    int dy,
    const GridIndex & goal,
    GridIndex & jump_point) const;
```

它会沿指定方向不断前进，直到出现以下情况：

1. 到达地图外；
2. 遇到障碍物；
3. 到达终点；
4. 遇到强迫邻居；
5. 斜向移动时发现水平或垂直方向存在跳点。

---

## 10.4 JPS搜索过程
```latex
起点加入开放队列
    ↓
取出优先级最低的跳点
    ↓
向八个方向执行jump
    ↓
跳跃过程中检查障碍物
    ↓
找到终点或强迫邻居
    ↓
把新的跳点加入开放队列
    ↓
记录跳点父节点
    ↓
到达终点后回溯路径
```

---

## 10.5 JPS特点
优点：

+ 在规则栅格地图中可以大幅减少扩展节点；
+ 路径结果与 A* 接近；
+ 长直线和开阔区域中速度优势明显。

缺点：

+ 实现比 A* 复杂；
+ 强迫邻居判断容易写错；
+ 不规则代价地图中不一定适用。

---

# 十一、PRM算法
## 11.1 PRM是什么
PRM 全称为 Probabilistic Roadmap，即概率路线图。

它分为两个主要阶段：

```latex
第一阶段：随机采样并建立路线图
第二阶段：在路线图上搜索路径
```

---

## 11.2 PRM实现步骤
### 第一步：采样自由节点
在地图中随机选择没有障碍物的格子：

```latex
·       ·
    ·
         ·
  ·
```

默认采样数量：

```latex
prm.sample_count = 500
```

### 第二步：寻找近邻
对每一个采样点寻找距离最近的若干节点。

默认近邻数：

```latex
prm.k_neighbors = 15
```

### 第三步：视线检测
两个采样点距离较近，不代表它们之间一定能够直接通行。

程序使用 Bresenham 栅格直线遍历，逐格检查连线经过的位置。

```latex
A · · · · B
```

算法会从 A 到 B 检查沿途所有格子：

```latex
A → 格子1 → 格子2 → 格子3 → B
```

只要有一个格子是障碍物：

```latex
A → □ → ■ → □ → B
```

这条边就不能加入路线图。

对应函数为：

```cpp
bool lineTraversable(
    const GridIndex & from,
    const GridIndex & to) const;
```

### 第四步：建立路线图
把通过视线检测的采样点连接起来：

```latex
·────·
│  ╱ │
·────·────·
```

### 第五步：图上搜索
将起点和终点连接到路线图后，在路线图上使用 Dijkstra 搜索最短路径。

---

## 11.3 PRM完整流程
```latex
随机采样自由格
    ↓
加入起点和终点
    ↓
计算采样点之间的距离
    ↓
为每个节点选择K个近邻
    ↓
执行视线碰撞检测
    ↓
无碰撞则建立边
    ↓
在路线图上运行Dijkstra
    ↓
回溯最终路径
```

---

## 11.4 PRM参数
| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `prm.sample_count` | 500 | 随机采样点数量 |
| `prm.k_neighbors` | 15 | 每个节点尝试连接的近邻数 |
| `prm.seed` | 23 | 随机种子 |


采样点太少时，路线图可能不连通。

采样点太多时，构图和碰撞检测时间会增加。

---

# 十二、RRT算法
## 12.1 RRT是什么
RRT 全称为 Rapidly-exploring Random Tree，即快速扩展随机树。

它从起点开始，通过随机采样不断扩展一棵树，直到树能够连接终点。

---

## 12.2 RRT基本步骤
### 第一步：随机采样
在地图中随机生成一个目标采样点：

```latex
q_random
```

### 第二步：寻找最近节点
在已有搜索树中寻找距离采样点最近的节点：

```latex
q_nearest
```

### 第三步：向采样点扩展
从最近节点向采样点方向移动一定距离：

```latex
q_nearest → q_new
```

这个过程称为 `Steer`。

### 第四步：碰撞检测
检查：

```latex
q_nearest → q_new
```

之间是否经过障碍物。

如果发生碰撞，就放弃这次扩展。

### 第五步：加入树
如果没有碰撞：

```latex
parent[q_new] = q_nearest
```

然后将新节点和新边加入搜索树。

### 第六步：尝试连接终点
如果新节点距离终点足够近，并且与终点之间没有障碍物，就完成搜索。

---

## 12.3 目标偏置
如果每次都完全随机采样，树可能很久都无法向终点扩展。

因此程序使用目标偏置：

```cpp
if (random_probability(random_generator) < goal_bias)
{
  random_sample = goal;
}
```

默认参数：

```latex
rrt.goal_bias = 0.12
```

含义是：

> 每次采样有 12% 的概率直接选择终点。
>

---

## 12.4 RRT参数
| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `rrt.max_iterations` | 8000 | 最大迭代次数 |
| `rrt.step_size` | 5.0格 | 每次最大扩展距离 |
| `rrt.goal_bias` | 0.12 | 直接采样终点的概率 |
| `rrt.seed` | 31 | 随机种子 |


地图分辨率为 0.2 m，因此 5 个格子约等于：

```latex
5 × 0.2 = 1 m
```

---

## 12.5 RRT特点
优点：

+ 容易在复杂空间中快速找到可行路径；
+ 不需要遍历整个栅格地图；
+ 适合高维空间。

缺点：

+ 路径通常不是最优路径；
+ 结果受随机种子影响；
+ 生成的路径可能弯曲；
+ 在狭窄通道中可能难以采样成功。

---

# 十三、RRT*算法
## 13.1 RRT*与RRT的区别
RRT 的目标是：

> 尽快找到一条能到终点的路径。
>

RRT* 的目标是：

> 在找到路径的同时，不断优化树的连接关系，使路径逐渐变短。
>

RRT* 在 RRT 基础上增加两个关键步骤：

1. 选择代价最低的父节点；
2. 重新连接附近节点。

---

## 13.2 选择最优父节点
RRT 直接选择最近节点作为父节点：

```latex
q_new.parent = q_nearest
```

RRT* 会检查新节点附近的多个节点：

```latex
q_near_1
q_near_2
q_near_3
```

然后计算：

```latex
起点到候选父节点的代价
+
候选父节点到新节点的距离
```

选择总代价最小的节点作为父节点。

---

## 13.3 重新连接
加入新节点后，RRT* 会检查附近节点：

> 如果附近节点改为经过新节点，总代价是否会更短？
>

如果会更短，就修改其父节点。

```latex
修改前：

A ─────────→ C
 \
  └──→ B

修改后：

A → B → C
```

这个过程称为 `Rewire`。

---

## 13.4 RRT*优化程度在哪里调整
影响 RRT* 优化程度的主要参数为：

```latex
rrt_star.max_iterations
rrt_star.rewire_radius
```

其中最直接的是：

```latex
rrt_star.max_iterations
```

迭代次数越大：

+ 采样次数越多；
+ 可以尝试更多连接；
+ 通常路径更短；
+ 计算时间更长。

重新连接半径：

```latex
rrt_star.rewire_radius
```

半径越大：

+ 每次检查的邻居更多；
+ 优化机会更多；
+ 单次迭代计算量更大。

---

## 13.5 RRT*参数
| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `rrt_star.max_iterations` | 5000 | 最大迭代次数 |
| `rrt_star.step_size` | 5.0格 | 单次扩展步长 |
| `rrt_star.rewire_radius` | 12.0格 | 选择父节点和重连的邻域半径 |
| `rrt_star.goal_bias` | 0.12 | 目标偏置 |
| `rrt_star.seed` | 37 | 随机种子 |


---

# 十四、Kinodynamic-RRT*
## 14.1 基本概念
普通 RRT* 的节点通常只包含位置：

```latex
(x, y)
```

Kinodynamic-RRT* 的节点还包含速度：

```latex
(x, y, vx, vy)
```

控制输入为加速度：

```latex
(ax, ay)
```

因此，它不仅考虑“几何上能不能走”，还考虑“机器人按照运动规律能不能到达”。

---

## 14.2 双积分运动模型
程序使用简化的双积分模型。

位置更新：

```latex
x_new = x + vx × dt + 0.5 × ax × dt²
y_new = y + vy × dt + 0.5 × ay × dt²
```

速度更新：

```latex
vx_new = vx + ax × dt
vy_new = vy + ay × dt
```

同时限制：

+ 最大速度；
+ 最大加速度；
+ 时间步长；
+ 碰撞；
+ 重连半径。

---

## 14.3 参数
| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `kinodynamic_rrt_star.max_iterations` | 6000 | 最大迭代次数 |
| `kinodynamic_rrt_star.time_step` | 0.8 | 单次状态推进时间 |
| `kinodynamic_rrt_star.max_speed` | 6.0 | 最大速度 |
| `kinodynamic_rrt_star.max_acceleration` | 4.0 | 最大加速度 |
| `kinodynamic_rrt_star.rewire_radius` | 10.0 | 重连半径 |
| `kinodynamic_rrt_star.goal_bias` | 0.15 | 目标偏置 |
| `kinodynamic_rrt_star.seed` | 41 | 随机种子 |


---

# 十五、Anytime-RRT*
## 15.1 基本概念
Anytime 算法的特点是：

> 先尽快得到一条可行路径，然后利用剩余时间继续优化。
>

因此它可以在不同时间返回不同质量的结果：

```latex
时间较少：返回一条可行路径
时间增加：路径逐渐变短
时间结束：返回当前找到的最好路径
```

---

## 15.2 实现过程
```latex
运行RRT*搜索
    ↓
找到第一条可行路径
    ↓
保存当前最好路径代价
    ↓
继续采样和重连
    ↓
剪掉不可能优于当前路径的分支
    ↓
时间或迭代次数用完
    ↓
返回保存的最好路径
```

---

## 15.3 参数
| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `anytime_rrt_star.max_iterations` | 12000 | 最大迭代次数 |
| `anytime_rrt_star.time_budget_ms` | 200 ms | 总时间预算 |
| `anytime_rrt_star.step_size` | 5.0格 | 扩展步长 |
| `anytime_rrt_star.rewire_radius` | 12.0格 | 重连半径 |
| `anytime_rrt_star.goal_bias` | 0.12 | 目标偏置 |
| `anytime_rrt_star.seed` | 43 | 随机种子 |


---

# 十六、Informed RRT*
## 16.1 基本概念
普通 RRT* 即使找到了一条路径，仍然会在整张地图中随机采样。

Informed RRT* 在找到第一条路径后，将采样区域缩小到一个椭圆内。

椭圆的两个焦点是：

+ 起点；
+ 终点。

椭圆长轴由当前最好路径长度决定。

---

## 16.2 为什么椭圆内部更有意义
假设当前最好路径长度为：

```latex
c_best
```

对于一个可能改善路径的点，它必须满足：

```latex
起点到该点的距离
+
该点到终点的距离
<
c_best
```

满足这个条件的点构成一个椭圆区域。

椭圆外面的点不可能产生更短路径，因此不再优先采样。

---

## 16.3 搜索过程
```latex
未找到路径
    ↓
在整张地图随机采样
    ↓
找到第一条可行路径
    ↓
得到当前最好路径长度
    ↓
建立以起终点为焦点的椭圆
    ↓
只在椭圆内采样
    ↓
继续选择父节点和重连
```

---

## 16.4 参数
| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `informed_rrt_star.max_iterations` | 7000 | 最大迭代次数 |
| `informed_rrt_star.step_size` | 5.0格 | 扩展步长 |
| `informed_rrt_star.rewire_radius` | 12.0格 | 重连半径 |
| `informed_rrt_star.goal_bias` | 0.12 | 目标偏置 |
| `informed_rrt_star.seed` | 47 | 随机种子 |


---

# 十七、RViz2可视化
## 17.1 颜色说明
当前项目中的主要可视化颜色如下：

| 内容 | 颜色 |
| --- | --- |
| 起点 | 绿色 |
| 终点 | 紫色 |
| A*路径 | 红色 |
| A*访问节点 | 橙色 |
| TimeBreak A*路径 | 粉红色 |
| TimeBreak A*访问节点 | 紫色 |
| Dijkstra路径 | 蓝色 |
| JPS路径 | 绿色 |
| JPS跳点 | 亮青色 |
| PRM路线图 | 灰色 |
| PRM路径 | 洋红色 |
| RRT搜索树 | 棕色 |
| RRT路径 | 橙色 |
| RRT*搜索树 | 蓝绿色 |
| RRT*路径 | 亮黄色 |
| Kinodynamic-RRT*路径 | 青色 |
| Anytime-RRT*路径 | 粉色 |
| Informed RRT*路径 | 亮绿色 |


> 【建议插图】在这里插入 RViz2 完整运行截图，并用箭头标出地图、起点、终点、路径和参数面板。
>

---

## 17.2 设置终点
启动程序后，在 RViz2 顶部工具栏中选择：

```latex
3D Nav Goal
```

也可以按快捷键：

```latex
G
```

然后在地图中：

1. 鼠标左键按下；
2. 拖动选择方向；
3. 松开鼠标；
4. 所有算法使用新的终点重新规划。

二维规划器只读取 X、Y 坐标。

---

## 17.3 地面小格子
地图分辨率为：

```latex
0.2 m
```

因此 RViz2 中的地面也使用：

```latex
0.2 m × 0.2 m
```

的小格子。

障碍物、起点、终点和 JPS 跳点的平面大小均保持为普通栅格大小，避免大方块遮挡周围地图。

---

## 17.4 参数实时调整
RViz2 左侧的“全部算法参数”面板可以实时调整：

+ 地图占据阈值；
+ 未知区域是否允许通行；
+ A*启发权重；
+ TimeBreak A*启发权重；
+ TimeBreak A*时间限制；
+ JPS启发权重；
+ PRM采样数量；
+ PRM近邻数量；
+ PRM随机种子；
+ RRT全部参数；
+ RRT*全部参数；
+ Kinodynamic-RRT*全部参数；
+ Anytime-RRT*全部参数；
+ Informed RRT*全部参数。

停止输入约 250 ms 后，面板会把参数发送给：

```latex
/demo_node_2d
```

规划节点接收参数后，会使用当前起点和终点重新规划。

Dijkstra 没有启发函数和随机参数，因此没有独立的实时调整参数。

---

# 十八、随机地图
## 18.1 地图随机种子
默认启动参数为：

```latex
map_seed = -1
```

`-1` 表示每次重新启动 Launch 都生成一张新地图。

运行：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py
```

每次重新启动后，地图应该不同。

---

## 18.2 固定地图
如果需要复现同一张地图，可以指定固定随机种子：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py map_seed:=11
```

只要参数和程序不变，使用相同种子就会生成相同地图。

---

## 18.3 为什么同一次运行中地图不变化
地图节点会重复发布同一张已经生成的地图，而不是每次发布时重新随机生成。

这样可以避免：

+ 规划过程中障碍物突然改变；
+ 路径和地图不一致；
+ RViz2显示不断跳动；
+ 不同算法使用了不同地图。

需要生成新地图时，应停止并重新运行 Launch。

---

# 十九、计算时间输出
每次规划时，程序会统计十种算法的计算时间，单位为毫秒。

计时基本形式为：

```cpp
const auto begin =
    std::chrono::steady_clock::now();

SearchResult result =
    searcher.searchAStar(start, goal);

const auto end =
    std::chrono::steady_clock::now();

const double elapsed_ms =
    std::chrono::duration<double, std::milli>(
        end - begin).count();
```

计时范围主要包含算法本身，不包含 RViz2 消息生成和显示时间。

PRM 的计算时间包含：

+ 随机采样；
+ 近邻计算；
+ 路线图构建；
+ 视线碰撞检测；
+ 图上 Dijkstra 搜索。

采样类算法受到随机种子、迭代次数和地图结构影响，因此同一算法在不同地图上的耗时可能不同。

---

# 二十、算法对比
| 算法 | 是否使用启发函数 | 是否随机 | 最优性 | 主要特点 |
| --- | ---: | ---: | --- | --- |
| DFS | 否 | 否 | 不保证 | 一直向深处搜索 |
| BFS | 否 | 否 | 单位代价下最优 | 按层搜索 |
| Dijkstra | 否 | 否 | 最优 | 从起点向四周扩散 |
| A* | 是 | 否 | 标准条件下最优 | 使用终点方向引导搜索 |
| TimeBreak A* | 是 | 否 | 未超时时同A* | 增加时间预算限制 |
| JPS | 是 | 否 | 规则条件下最优 | 跳过大量对称节点 |
| PRM | 图搜索阶段可最优 | 是 | 依赖路线图 | 先构图，再查询路径 |
| RRT | 否 | 是 | 不保证 | 快速寻找可行路径 |
| RRT* | 否 | 是 | 渐近最优 | 选择父节点并重连 |
| Kinodynamic-RRT* | 否 | 是 | 渐近改进 | 考虑速度和加速度 |
| Anytime-RRT* | 否 | 是 | 持续改进 | 有剩余时间就继续优化 |
| Informed RRT* | 使用路径代价约束采样 | 是 | 渐近最优 | 首解后在椭圆内采样 |


---

# 二十一、编译与运行
## 21.1 编译工作空间
打开终端：

```bash
cd ~/ws11
source /opt/ros/humble/setup.bash
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

其中：

```latex
--symlink-install
```

方便修改配置文件、Launch 文件和 Python 文件后快速生效。

```latex
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

用于生成 `compile_commands.json`，帮助 VS Code 正确识别 ROS 2 和 C++ 头文件。

---

## 21.2 加载当前工作空间
编译成功后执行：

```bash
source ~/ws11/install/setup.bash
```

如果当前目录就是 `~/ws11`，也可以写成：

```bash
source install/setup.bash
```

或者：

```bash
. install/setup.bash
```

这三种写法的作用相同。

其中：

```latex
source
```

和：

```latex
.
```

都表示在当前终端中执行脚本。

---

## 21.3 启动程序
```bash
ros2 launch grid_path_searcher_2d demo_launch.py
```

启动文件会同时运行：

+ `random_map_2d`
+ `demo_node_2d`
+ `waypoint_generator`
+ `rviz2`

因此不需要再单独运行 `rviz2`。

---

## 21.4 不启动RViz2
如果只想运行节点：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py use_rviz:=false
```

---

## 21.5 启动固定地图
```bash
ros2 launch grid_path_searcher_2d demo_launch.py map_seed:=11
```

---

# 二十二、环境加载说明
## 22.1 是否每次都要执行`source`
如果终端没有自动加载 ROS 2 和工作空间环境，就需要执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws11/install/setup.bash
```

第一行加载 ROS 2 Humble。

第二行加载 `ws11` 中编译完成的功能包。

如果没有加载 `ws11`，可能出现：

```latex
Package 'grid_path_searcher_2d' not found
```

---

## 22.2 使用alias
可以在 `~/.bashrc` 中添加：

```bash
alias sws11='source /opt/ros/humble/setup.bash && source ~/ws11/install/setup.bash'
```

保存后执行：

```bash
source ~/.bashrc
```

以后打开新终端，只需要运行：

```bash
sws11
```

即可加载 ROS 2 和 `ws11`。

这种方式不会改变 ROS 2 安装，也不会修改系统环境，只是在当前终端中加载相应工作空间。

---

# 二十三、VS Code配置
## 23.1 为什么会提示找不到头文件
例如：

```latex
无法打开源文件 "nav_msgs/msg/odometry.hpp"
```

通常不是程序真的无法编译，而是 VS Code 没有找到 ROS 2 的头文件路径或编译数据库。

---

## 23.2 生成编译数据库
在 `ws11` 中重新编译：

```bash
cd ~/ws11
source /opt/ros/humble/setup.bash
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

生成位置通常为：

```latex
~/ws11/build/compile_commands.json
```

如果使用多个包，实际的编译数据库也可能位于各个包对应的构建目录中。

---

## 23.3 VS Code终端能否达到相同效果
VS Code 集成终端和普通 Ubuntu 终端本质上都可以运行 Bash，因此可以使用相同命令：

```bash
cd ~/ws11
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch grid_path_searcher_2d demo_launch.py
```

只要：

+ 图形显示环境正常；
+ ROS 2 环境已加载；
+ `ws11` 已编译；
+ 当前终端加载了 `install/setup.bash`；

就可以从 VS Code 终端正常启动 RViz2。

---

# 二十四、实际问题与解决过程
## 24.1 找不到功能包
报错形式：

```latex
Package 'grid_path_searcher_2d' not found
```

可能原因：

1. 没有编译；
2. 编译失败；
3. 没有加载 `ws11/install/setup.bash`；
4. 当前加载的是其他工作空间；
5. 功能包名称输入错误。

解决方法：

```bash
cd ~/ws11
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep grid_path_searcher_2d
```

---

## 24.2 RViz2意外退出
RViz2 意外退出不一定表示路径规划源码已经损坏。

首先观察启动终端是否还有以下节点：

```bash
ros2 node list
```

如果只有 RViz2 退出，可以重新启动 Launch。

如果反复退出，应检查：

+ RViz2终端错误信息；
+ 显卡驱动；
+ OpenGL；
+ RViz配置文件；
+ 自定义面板插件；
+ Marker数据是否合法。

---

## 24.3 地图每次都一样
检查是否使用了固定种子：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py map_seed:=11
```

如果指定了固定种子，地图每次相同是正常现象。

需要随机地图时使用：

```bash
ros2 launch grid_path_searcher_2d demo_launch.py map_seed:=-1
```

同一次运行中地图保持不变也是正常现象。

---

## 24.4 VS Code提示`compile_commands.json`没有源文件
提示形式：

```latex
在 build/compile_commands.json 中未找到 demo_node.cpp
```

需要确认：

1. 当前 VS Code 打开的是 `~/ws11`；
2. 已使用 `CMAKE_EXPORT_COMPILE_COMMANDS=ON` 编译；
3. `demo_node.cpp` 已加入 `CMakeLists.txt`；
4. VS Code 配置指向正确的编译数据库；
5. 修改文件后重新执行过 `colcon build`。

---

## 24.5 参数修改后没有重新规划
当前面板停止输入约 250 ms 后才会发送参数。

如果没有重新规划，应检查：

```bash
ros2 param list /demo_node_2d
```

也可以查看某个参数：

```bash
ros2 param get /demo_node_2d rrt_star.max_iterations
```

检查规划节点是否存在：

```bash
ros2 node list
```

---

## 24.6 设置目标后没有路径
可能原因：

+ 终点位于障碍物内部；
+ 终点超出地图；
+ 起点与终点之间不存在通路；
+ 随机采样算法迭代次数不足；
+ 目标附近没有可连接区域；
+ TimeBreak A*时间限制太短。

可以尝试：

+ 重新选择终点；
+ 增大迭代次数；
+ 增大 TimeBreak A*时间限制；
+ 调整 PRM采样数量；
+ 调整 RRT目标偏置；
+ 重新生成地图。

---

# 二十五、暂时只做了解的内容
以下内容目前先了解概念，不继续加入当前代码。

## 25.1 Cross-Entropy Motion Planning
使用概率分布生成候选轨迹，根据表现较好的样本不断更新采样分布，使后续采样逐渐集中到高质量区域。

---

## 25.2 LBTRRT
全称为 Lower Bound Tree RRT。

它在路径质量和计算速度之间进行权衡，并使用近似最优性的界限控制结果。

---

## 25.3 SST
全称为 Sparse Stable Trees。

它通过稀疏化搜索树减少节点数量，适合动力学约束下的运动规划。

---

## 25.4 T-RRT
全称为 Transition-based RRT。

它将代价地图与随机树结合，通过类似概率接受机制限制搜索树进入高代价区域。

---

## 25.5 Vector Field RRT
在 RRT 扩展过程中加入向量场，使树的生长方向受到环境或运动方向引导。

---

## 25.6 Parallel RRT
同时运行多棵随机树或者并行执行采样和碰撞检测，从而提高搜索速度。

---

## 25.7 OMPL
OMPL 全称为 Open Motion Planning Library。

它提供大量采样式路径规划算法，例如：

+ RRT
+ RRTConnect
+ RRT*
+ PRM
+ KPIECE
+ EST

OMPL 主要负责路径规划算法，不直接负责机器人控制和可视化。

---

## 25.8 MoveIt 2
MoveIt 2 是 ROS 2 中用于机械臂运动规划的重要框架。

它可以结合：

+ 机器人模型；
+ 碰撞检测；
+ 运动学；
+ OMPL；
+ 轨迹规划；
+ 机械臂控制。

当前二维移动机器人路径规划项目暂时不需要加入 MoveIt 2。

---

## 25.9 Navigation Stack与Nav2
ROS 1 中通常称为 Navigation Stack。

ROS 2 中主要使用 Nav2。

Nav2包含：

+ 地图服务；
+ 定位；
+ 全局规划器；
+ 局部控制器；
+ 代价地图；
+ 行为树；
+ 路径跟踪；
+ 恢复行为。

当前项目主要学习的是 Nav2 中“全局路径规划”相关的底层算法思想。

---

## 25.10 局部规划算法
后续可以继续了解：

+ DWA
+ TEB
+ E-Band

这些算法需要进一步考虑：

+ 机器人速度；
+ 机器人角速度；
+ 运动学约束；
+ 动态障碍物；
+ 局部避障；
+ 路径跟踪。

---

# 二十六、学习总结
通过当前二维路径规划项目，可以把不同算法分成两类。

## 栅格搜索算法
包括：

+ Dijkstra
+ A*
+ TimeBreak A*
+ JPS

它们直接在地图小格子之间搜索，结果稳定，适合二维栅格地图。

其中：

+ Dijkstra只使用真实代价；
+ A*增加终点方向；
+ TimeBreak A*增加时间预算；
+ JPS通过跳点减少无意义扩展。

## 采样规划算法
包括：

+ PRM
+ RRT
+ RRT*
+ Kinodynamic-RRT*
+ Anytime-RRT*
+ Informed RRT*

它们不会逐个遍历所有格子，而是在空间中采样节点。

其中：

+ PRM先建立路线图；
+ RRT快速寻找可行路径；
+ RRT*通过重连不断优化；
+ Kinodynamic-RRT*考虑速度和加速度；
+ Anytime-RRT*在剩余时间内持续改进；
+ Informed RRT*找到路径后缩小采样区域。

整个程序的核心运行逻辑可以总结为：

```latex
生成地图
    ↓
接收起点和终点
    ↓
把世界坐标转换为栅格坐标
    ↓
运行不同路径规划算法
    ↓
记录路径、访问节点、跳点和搜索树
    ↓
统计计算时间
    ↓
转换回世界坐标
    ↓
发布ROS 2消息
    ↓
RViz2显示结果
    ↓
调整参数或设置新终点
    ↓
重新规划
```

这个项目已经完成了从地图生成、算法实现、参数控制到 RViz2 可视化的一整套二维路径规划学习流程。
