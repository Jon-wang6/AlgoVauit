/**
 * @file graph_searcher.cpp
 * @brief graph_searcher.hpp 中三维 A* 与 JPS 搜索器的算法实现。
 *
 * 文件职责：管理三维占据栅格和搜索节点，完成坐标转换、启发式代价计算、
 * A* 与 JPS 邻居扩展、跳点递归、强迫邻居判断及最终路径回溯。
 *
 * 包含的类：gridPathFinder、JPS3DNeib（声明位于 graph_searcher.hpp）。
 * 包含的主要函数：
 * - initGridMap()/resetGrid()/resetUsedGrids()/setObs()：地图和搜索状态管理。
 * - getDiagHeu()/getManhHeu()/getEuclHeu()/getHeu()：启发式代价计算。
 * - retrievePath()/getPath()/getVisitedNodes()/getCloseNodes()：结果提取。
 * - gridIndex2coord()/coord2gridIndex()/coordRounding()：坐标转换。
 * - isOccupied()/isFree()：栅格占据状态查询。
 * - getSucc()/getJpsSucc()：A* 与 JPS 后继节点生成。
 * - graphSearch()：统一搜索主循环。
 * - jump()/hasForced()：JPS 跳点与强迫邻居检测。
 * - JPS3DNeib()/Neib()/FNeib()：生成三维 JPS 邻居查找表。
 */
#include "graph_searcher.hpp"

#include <chrono>
#include <iostream>

using namespace std;
using namespace Eigen;

/**
 * 初始化搜索地图：保存边界与分辨率，创建占据数组，并为每个三维索引分配 GridNode。
 * 后续搜索只重置实际访问过的节点，避免每次遍历整个三维数组。
 */
void gridPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

/** 将节点恢复为“未访问”状态，清除父节点和累计代价。 */
void gridPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

/** 清理上一轮打开集、关闭集和路径标记，为下一次 A* 或 JPS 搜索复用地图。 */
void gridPathFinder::resetUsedGrids()
{   
    //ROS_WARN("expandedNodes size : %d", expandedNodes.size()); 
    for(auto tmpPtr:expandedNodes)
        resetGrid(tmpPtr);

    GridNodePtr tmpPtr = NULL;
    for(auto ptr:openSet){   
        tmpPtr = ptr.second;
        resetGrid(tmpPtr);
    }

    for(auto ptr:gridPath)
        ptr->is_path = false;

    expandedNodes.clear();
}

/** 把世界坐标转换为一维数组索引并标记占据；地图外坐标直接忽略。 */
void gridPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

/** 三维对角距离启发函数，按直线、平面对角线和空间对角线组合最短代价。 */
double gridPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

/** 曼哈顿距离启发函数，适用于只沿坐标轴移动的代价估计。 */
double gridPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

/** 欧氏距离启发函数。 */
double gridPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{   
    return (node2->index - node1->index).norm();
}

/** 当前统一启发函数入口；微小 tie_breaker 用于减少相同 fScore 的搜索分支。 */
double gridPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker * getDiagHeu(node1, node2);
    //return getEuclHeu(node1, node2);
}

/** 从终点沿 cameFrom 指针回溯到起点，同时给最终路径节点加上 is_path 标记。 */
vector<GridNodePtr> gridPathFinder::retrievePath(GridNodePtr current)
{   
    vector<GridNodePtr> path;
    path.push_back(current);

    while(current->cameFrom != NULL)
    {   
        current->is_path = true;
        current = current -> cameFrom;
        path.push_back(current);
    }

    current->is_path = true;

    return path;
}

/** 扫描节点表并返回处于打开集或关闭集中的所有访问节点，主要用于调试。 */
vector<Vector3d> gridPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++)
            {   
                if(GridNodeMap[i][j][k]->id != 0)
                //if(GridNodeMap[i][j][k]->id == -1)
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    std::clog << "visited_nodes size: " << visited_nodes.size() << '\n';
    return visited_nodes;
}

/** 返回已经扩展但不属于最终路径的关闭节点，用于 RViz 蓝色搜索区域。 */
vector<Vector3d> gridPathFinder::getCloseNodes()
{   
    vector<Vector3d> vec;
    for(auto tmpPtr:expandedNodes)
    {   
        if( !tmpPtr->is_path )
            vec.push_back(tmpPtr->coord);
    }

    return vec;
}

/** 将内部 GridNode 路径转换为坐标，并翻转成起点到终点的顺序。 */
vector<Vector3d> gridPathFinder::getPath() 
{   
    vector<Vector3d> path;

    for(auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());
    return path;
}

/** 栅格索引转为对应单元中心的世界坐标。 */
inline Vector3d gridPathFinder::gridIndex2coord(const Vector3i & index) const
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/** 世界坐标转栅格索引，并将结果钳制在有效地图范围内。 */
inline Vector3i gridPathFinder::coord2gridIndex(const Vector3d & pt) const
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

/** 通过“坐标→索引→坐标”把任意位置吸附到栅格中心。 */
Eigen::Vector3d gridPathFinder::coordRounding(const Eigen::Vector3d & coord) const
{
    return gridIndex2coord(coord2gridIndex(coord));
}

/** Vector3i 形式的占据查询。 */
inline bool gridPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

/** Vector3i 形式的自由空间查询。 */
inline bool gridPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

/** 三个整数索引形式的占据查询；只有边界内且 data=1 才返回 true。 */
inline bool gridPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

/** 三个整数索引形式的自由查询；越界或已占据均返回 false。 */
inline bool gridPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

/**
 * 生成 JPS 后继：查自然邻居与强迫邻居表，对候选方向执行 jump，
 * 仅保留真正的跳点并计算当前节点到跳点的欧氏边代价。
 */
inline void gridPathFinder::getJpsSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets, int num_iter)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    int num_neib  = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Vector3i neighborIdx;
        Vector3i expandDir;

        if( dev < num_neib) {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        else {
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            if( isOccupied(nx, ny, nz) ) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }

        if( num_iter == 1 )
            debugNodes.push_back(gridIndex2coord(neighborIdx));

        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;
        
        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}

/** 普通 A* 后继生成：枚举当前位置周围 3×3×3 中除自身外的 26 个邻居。 */
inline void gridPathFinder::getSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    Vector3i neighborIdx;
    for(int dx = -1; dx < 2; dx++){
        for(int dy = -1; dy < 2; dy++){
            for(int dz = -1; dz < 2; dz++){
                
                if( dx == 0 && dy == 0 && dz ==0 )
                    continue; 

                neighborIdx(0) = (currentPtr -> index)(0) + dx;
                neighborIdx(1) = (currentPtr -> index)(1) + dy;
                neighborIdx(2) = (currentPtr -> index)(2) + dz;

                if(    neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE
                    || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE
                    || neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE){
                    continue;
                }

                neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
                edgeCostSets.   push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

/**
 * A* 与 JPS 共用搜索主循环：
 * 1. 起终点离散到栅格并把起点加入 openSet；
 * 2. 每轮取 fScore 最小节点，命中目标则回溯路径；
 * 3. 根据 use_jps 生成剪枝后继或完整 26 邻域；
 * 4. 跳过障碍/关闭节点，对更优 gScore 执行松弛并更新父节点；
 * 5. 记录耗时、路径代价和扩展节点供可视化使用。
 */
void gridPathFinder::graphSearch(Vector3d start_pt, Vector3d end_pt, bool use_jps)
{   
    const auto time_1 = std::chrono::steady_clock::now();
    debugNodes.clear();

    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);

    goalIdx = end_idx;

    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    openSet.clear();

    GridNodePtr neighborPtr = NULL;
    GridNodePtr currentPtr  = NULL;

    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr, endPtr);
    startPtr -> id = 1; //put start node in open set
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // we only cover 3d case in this project.
    while ( !openSet.empty() )
    {   
        num_iter ++;
        currentPtr = openSet.begin() -> second;

        if( currentPtr->index == goalIdx )
        {
            const auto time_2 = std::chrono::steady_clock::now();
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(time_2 - time_1).count();

            if( use_jps )
                std::clog << "[JPS] success in " << elapsed_ms << " ms, path cost "
                          << currentPtr->gScore * resolution << " m\n";
            else
                std::clog << "[A*] success in " << elapsed_ms << " ms, path cost "
                          << currentPtr->gScore * resolution << " m\n";
            
            gridPath = retrievePath(currentPtr);
            return;
        }         
        openSet.erase(openSet.begin());
        currentPtr -> id = -1; //move current node from open set to closed set.
        expandedNodes.push_back(currentPtr);
        
        if(!use_jps)
            getSucc(currentPtr, neighborPtrSets, edgeCostSets);
        else
            getJpsSucc(currentPtr, neighborPtrSets, edgeCostSets, num_iter);

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            if( isOccupied(neighborPtr->index) || neighborPtr -> id == -1)
                continue;

            double edge_cost = edgeCostSets[i];            
            tentative_gScore = currentPtr -> gScore + edge_cost; 

            if(neighborPtr -> id != 1){ //discover a new node
                neighborPtr -> id        = 1;
                neighborPtr -> cameFrom  = currentPtr;
                neighborPtr -> gScore    = tentative_gScore;
                neighborPtr -> fScore    = neighborPtr -> gScore + getHeu(neighborPtr, endPtr); 
                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                continue;
            }
            else if(tentative_gScore <= neighborPtr-> gScore){ //in open set and need update
                neighborPtr -> cameFrom = currentPtr;
                neighborPtr -> gScore = tentative_gScore;
                neighborPtr -> fScore = tentative_gScore + getHeu(neighborPtr, endPtr); 
                openSet.erase(neighborPtr -> nodeMapIt);
                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.

                // if change its parents, update the expanding direction
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
            }
            
        }
    }

    const auto time_2 = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(time_2 - time_1).count();

    if(elapsed_s > 0.1)
        std::clog << "Path search failed after " << elapsed_s << " s\n";
}

/**
 * JPS 递归跳跃：前进一步后依次检查边界/障碍、目标、强迫邻居，
 * 对较低维子方向递归检查，最后继续沿原方向前进。
 */
bool gridPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    neiIdx = curIdx + expDir;

    if( !isFree(neiIdx) )
        return false;

    if( neiIdx == goalIdx )
        return true;

    if( hasForced(neiIdx, expDir) )
        return true;

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];

    for( int k = 0; k < num_neib - 1; ++k ){
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }

    return jump(neiIdx, expDir, neiIdx);
}

/** 按直线、二维对角或三维对角运动类型检查对应数量的强迫邻居。 */
inline bool gridPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch(norm1)
    {
        case 1:
            // 1-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        default:
            return false;
    }
}

constexpr int JPS3DNeib::nsz[4][2];
/** 遍历 27 个方向，预计算每种方向的自然邻居和强迫邻居规则。 */
JPS3DNeib::JPS3DNeib() 
{
    int id = 0;
    for(int dz = -1; dz <= 1; ++ dz) {
        for(int dy = -1; dy <= 1; ++ dy) {
            for(int dx = -1; dx <= 1; ++ dx) {
                int norm1 = abs(dx) + abs(dy) + abs(dz);
            
                for(int dev = 0; dev < nsz[norm1][0]; ++ dev)
                    Neib(dx,dy,dz,norm1,dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
            
                for(int dev = 0; dev < nsz[norm1][1]; ++ dev){
                    FNeib(dx,dy,dz,norm1,dev,
                    f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
                    f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
                }
                
                id ++;
            }
        }
    }
}


/** 根据运动维数 norm1 和编号 dev 返回一个自然邻居的方向分量。 */
void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
    switch(norm1)
    {
        case 0:
            switch(dev)
            {
                case 0: tx=1; ty=0; tz=0; return;
                case 1: tx=-1; ty=0; tz=0; return;
                case 2: tx=0; ty=1; tz=0; return;
                case 3: tx=1; ty=1; tz=0; return;
                case 4: tx=-1; ty=1; tz=0; return;
                case 5: tx=0; ty=-1; tz=0; return;
                case 6: tx=1; ty=-1; tz=0; return;
                case 7: tx=-1; ty=-1; tz=0; return;
                case 8: tx=0; ty=0; tz=1; return;
                case 9: tx=1; ty=0; tz=1; return;
                case 10: tx=-1; ty=0; tz=1; return;
                case 11: tx=0; ty=1; tz=1; return;
                case 12: tx=1; ty=1; tz=1; return;
                case 13: tx=-1; ty=1; tz=1; return;
                case 14: tx=0; ty=-1; tz=1; return;
                case 15: tx=1; ty=-1; tz=1; return;
                case 16: tx=-1; ty=-1; tz=1; return;
                case 17: tx=0; ty=0; tz=-1; return;
                case 18: tx=1; ty=0; tz=-1; return;
                case 19: tx=-1; ty=0; tz=-1; return;
                case 20: tx=0; ty=1; tz=-1; return;
                case 21: tx=1; ty=1; tz=-1; return;
                case 22: tx=-1; ty=1; tz=-1; return;
                case 23: tx=0; ty=-1; tz=-1; return;
                case 24: tx=1; ty=-1; tz=-1; return;
                case 25: tx=-1; ty=-1; tz=-1; return;
            }
            return;
        case 1:
            tx = dx; ty = dy; tz = dz; return;
        case 2:
            switch(dev){
                case 0:
                    if(dz == 0){
                        tx = 0; ty = dy; tz = 0; return;
                    }else{
                        tx = 0; ty = 0; tz = dz; return;
                    }
                case 1:
                    if(dx == 0){
                        tx = 0; ty = dy; tz = 0; return;
                    }else{
                        tx = dx; ty = 0; tz = 0; return;
                    }
                case 2:
                    tx = dx; ty = dy; tz = dz; return;
            }
            return;
        case 3:
            switch(dev){
                case 0: tx = dx; ty =  0; tz =  0; return;
                case 1: tx =  0; ty = dy; tz =  0; return;
                case 2: tx =  0; ty =  0; tz = dz; return;
                case 3: tx = dx; ty = dy; tz =  0; return;
                case 4: tx = dx; ty =  0; tz = dz; return;
                case 5: tx =  0; ty = dy; tz = dz; return;
                case 6: tx = dx; ty = dy; tz = dz; return;
            }
    }
}

/**
 * 生成强迫邻居规则：f=(fx,fy,fz) 是需要检查是否被占据的位置，
 * n=(nx,ny,nz) 是 f 被占据时必须加入搜索的邻居方向。
 * 分别处理轴向运动、平面对角运动和空间对角运动。
 */
void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
    switch(norm1)
    {
        case 1:
            switch(dev){
                case 0: fx= 0; fy= 1; fz = 0; break;
                case 1: fx= 0; fy=-1; fz = 0; break;
                case 2: fx= 1; fy= 0; fz = 0; break;
                case 3: fx= 1; fy= 1; fz = 0; break;
                case 4: fx= 1; fy=-1; fz = 0; break;
                case 5: fx=-1; fy= 0; fz = 0; break;
                case 6: fx=-1; fy= 1; fz = 0; break;
                case 7: fx=-1; fy=-1; fz = 0; break;
            }
            nx = fx; ny = fy; nz = dz;
            // switch order if different direction
            if(dx != 0){
                fz = fx; fx = 0;
                nz = fz; nx = dx;
            }

            if(dy != 0){
                fz = fy; fy = 0;
                nz = fz; ny = dy;
            }
            return;
        case 2:
            if(dx == 0){
                switch(dev){
                    case 0:
                        fx = 0; fy = 0; fz = -dz;
                        nx = 0; ny = dy; nz = -dz;
                        return;
                    case 1:
                        fx = 0; fy = -dy; fz = 0;
                        nx = 0; ny = -dy; nz = dz;
                        return;
                    case 2:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = dy; nz = dz;
                        return;
                    case 3:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = dy; nz = dz;
                        return;
                    case 4:
                        fx = 1; fy = 0; fz = -dz;
                        nx = 1; ny = dy; nz = -dz;
                        return;
                    case 5:
                        fx = 1; fy = -dy; fz = 0;
                        nx = 1; ny = -dy; nz = dz;
                        return;
                    case 6:
                        fx = -1; fy = 0; fz = -dz;
                        nx = -1; ny = dy; nz = -dz;
                        return;
                    case 7:
                        fx = -1; fy = -dy; fz = 0;
                        nx = -1; ny = -dy; nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = dy; nz = 0;
                        return;
                    case 9:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = 0; nz = dz;
                        return;
                    case 10:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = dy; nz = 0;
                        return;
                    case 11:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = 0; nz = dz;
                        return;
                }
            }
            else if(dy == 0){
                switch(dev){
                    case 0:
                        fx = 0; fy = 0; fz = -dz;
                        nx = dx; ny = 0; nz = -dz;
                        return;
                    case 1:
                        fx = -dx; fy = 0; fz = 0;
                        nx = -dx; ny = 0; nz = dz;
                        return;
                    case 2:
                        fx = 0; fy = 1; fz = 0;
                        nx = dx; ny = 1; nz = dz;
                        return;
                    case 3:
                        fx = 0; fy = -1; fz = 0;
                        nx = dx; ny = -1;nz = dz;
                        return;
                    case 4:
                        fx = 0; fy = 1; fz = -dz;
                        nx = dx; ny = 1; nz = -dz;
                        return;
                    case 5:
                        fx = -dx; fy = 1; fz = 0;
                        nx = -dx; ny = 1; nz = dz;
                        return;
                    case 6:
                        fx = 0; fy = -1; fz = -dz;
                        nx = dx; ny = -1; nz = -dz;
                        return;
                    case 7:
                        fx = -dx; fy = -1; fz = 0;
                        nx = -dx; ny = -1; nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 0; fy = 1; fz = 0;
                        nx = dx; ny = 1; nz = 0;
                        return;
                    case 9:
                        fx = 0; fy = 1; fz = 0;
                        nx = 0; ny = 1; nz = dz;
                        return;
                    case 10:
                        fx = 0; fy = -1; fz = 0;
                        nx = dx; ny = -1; nz = 0;
                        return;
                    case 11:
                        fx = 0; fy = -1; fz = 0;
                        nx = 0; ny = -1; nz = dz;
                        return;
                }
            }
            else{// dz==0
                switch(dev){
                    case 0:
                        fx = 0; fy = -dy; fz = 0;
                        nx = dx; ny = -dy; nz = 0;
                        return;
                    case 1:
                        fx = -dx; fy = 0; fz = 0;
                        nx = -dx; ny = dy; nz = 0;
                        return;
                    case 2:
                        fx =  0; fy = 0; fz = 1;
                        nx = dx; ny = dy; nz = 1;
                        return;
                    case 3:
                        fx =  0; fy = 0; fz = -1;
                        nx = dx; ny = dy; nz = -1;
                        return;
                    case 4:
                        fx = 0; fy = -dy; fz = 1;
                        nx = dx; ny = -dy; nz = 1;
                        return;
                    case 5:
                        fx = -dx; fy = 0; fz = 1;
                        nx = -dx; ny = dy; nz = 1;
                        return;
                    case 6:
                        fx = 0; fy = -dy; fz = -1;
                        nx = dx; ny = -dy; nz = -1;
                        return;
                    case 7:
                        fx = -dx; fy = 0; fz = -1;
                        nx = -dx; ny = dy; nz = -1;
                        return;
                    // Extras
                    case 8:
                        fx =  0; fy = 0; fz = 1;
                        nx = dx; ny = 0; nz = 1;
                        return;
                    case 9:
                        fx = 0; fy = 0; fz = 1;
                        nx = 0; ny = dy; nz = 1;
                        return;
                    case 10:
                        fx =  0; fy = 0; fz = -1;
                        nx = dx; ny = 0; nz = -1;
                        return;
                    case 11:
                        fx = 0; fy = 0; fz = -1;
                        nx = 0; ny = dy; nz = -1;
                        return;
                }
            }
            return;
        case 3:
            switch(dev){
                case 0:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = dy; nz = dz;
                    return;
                case 1:
                    fx = 0; fy = -dy; fz = 0;
                    nx = dx; ny = -dy; nz = dz;
                    return;
                case 2:
                    fx = 0; fy = 0; fz = -dz;
                    nx = dx; ny = dy; nz = -dz;
                    return;
                // Need to check up to here for forced!
                case 3:
                    fx = 0; fy = -dy; fz = -dz;
                    nx = dx; ny = -dy; nz = -dz;
                    return;
                case 4:
                    fx = -dx; fy = 0; fz = -dz;
                    nx = -dx; ny = dy; nz = -dz;
                    return;
                case 5:
                    fx = -dx; fy = -dy; fz = 0;
                    nx = -dx; ny = -dy; nz = dz;
                    return;
                // Extras
                case 6:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = 0; nz = dz;
                    return;
                case 7:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = dy; nz = 0;
                    return;
                case 8:
                    fx = 0; fy = -dy; fz = 0;
                    nx = 0; ny = -dy; nz = dz;
                    return;
                case 9:
                    fx = 0; fy = -dy; fz = 0;
                    nx = dx; ny = -dy; nz = 0;
                    return;
                case 10:
                    fx = 0; fy = 0; fz = -dz;
                    nx = 0; ny = dy; nz = -dz;
                    return;
                case 11:
                    fx = 0; fy = 0; fz = -dz;
                    nx = dx; ny = 0; nz = -dz;
                    return;
            }
    }
}
