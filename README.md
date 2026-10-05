# AlgoVauit

算法学习、实现与可视化实验仓库。`main` 分支仅作为导航首页，各主题保存在独立分支中。

## 分支导航

| 分支 | 对应目录 | 主要内容 |
| --- | --- | --- |
| [`path-planning`](https://github.com/Jon-wang6/AlgoVauit/tree/path-planning) | `~/ws11` | ROS 2 二维路径规划、RViz2 可视化与算法性能对比 |

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

## 参考资料

- [ROS 2 二维路径规划算法学习与实现](https://www.yuque.com/g/jonwang-pfbbk/gd0so3/aisohqo8yfbk70np/collaborator/join?token=QXzK0gTytpligebR&source=doc_collaborator#)
