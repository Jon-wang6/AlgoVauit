# AlgoVauit

算法学习、实现与可视化实验仓库。`main` 分支仅作为导航首页，各主题保存在独立分支中。

## 分支导航

| 分支 | 对应目录 | 主要内容 |
| --- | --- | --- |
| [`path-planning`](https://github.com/Jon-wang6/AlgoVauit/tree/path-planning) | `~/ws11` | ROS 2 二维/三维路径规划、RViz2 可视化与算法性能对比 |

## 路径规划算法

`path-planning` 分支目前包含：

- A* 与 TimeBreak A*
- Dijkstra
- JPS
- PRM
- RRT、RRT*、Anytime RRT*、Informed RRT* 与 Kinodynamic RRT*
- 三维点云地图与二维 OccupancyGrid 演示
- RViz2 交互目标工具和实时参数面板

克隆该分支：

```bash
git clone --branch path-planning --single-branch https://github.com/Jon-wang6/AlgoVauit.git ws11
```

具体构建、启动和操作说明请查看对应分支中的 README。
