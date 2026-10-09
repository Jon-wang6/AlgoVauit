# AlgoVauit

算法学习、实现与可视化实验仓库。`main` 分支仅作为导航首页，各主题保存在独立分支中。

## 分支导航

| 分支 | 对应目录 | 主要内容 |
| --- | --- | --- |
| [`path-planning`](https://github.com/Jon-wang6/AlgoVauit/tree/path-planning) | `~/ws11` | ROS 2 二维路径规划、RViz2 可视化与算法性能对比 |
| [`robotics`](https://github.com/Jon-wang6/AlgoVauit/tree/robotics) | `notes/` | 空间位姿、旋转矩阵、齐次变换与机器人学基础 |

## 路径规划算法

`path-planning` 分支目前包含：

- A* 与 TimeBreak A*
- Dijkstra
- JPS
- PRM
- RRT、RRT*、Anytime RRT*、Informed RRT* 与 Kinodynamic RRT*
- 二维 OccupancyGrid 随机地图演示
- RViz2 交互目标工具和实时参数面板

仓库中保留早期三维路径规划示例代码，仅供参考。

克隆该分支：

```bash
git clone --branch path-planning --single-branch https://github.com/Jon-wang6/AlgoVauit.git ws11
```

具体构建、启动和操作说明请查看对应分支中的 README。

## 学习笔记

- [ROS 2 二维路径规划算法学习与实现](https://github.com/Jon-wang6/AlgoVauit/blob/path-planning/notes/ROS2二维路径规划算法学习与实现.md)
- [机器人空间位姿描述](https://github.com/Jon-wang6/AlgoVauit/blob/robotics/notes/机器人空间位姿描述.md)
