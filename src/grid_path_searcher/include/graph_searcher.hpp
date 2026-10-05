/**
 * @file graph_searcher.hpp
 * @brief 三维占据栅格 A* 与 Jump Point Search 搜索器的公开声明。
 *
 * 文件职责：定义搜索节点的数据结构、JPS 邻居查找表和 gridPathFinder
 * 接口；具体算法步骤在 src/graph_searcher.cpp 中实现。
 *
 * 包含的类/结构：
 * - JPS3DNeib：预计算自然邻居与强迫邻居方向。
 * - GridNode：保存单个栅格节点的索引、代价、状态和父节点。
 * - gridPathFinder：提供地图初始化、障碍写入、A* 与 JPS 搜索及结果查询。
 *
 * 包含的主要函数：initGridMap、setObs、graphSearch、getPath、
 * getVisitedNodes、getCloseNodes、坐标转换、占据查询、后继扩展、jump、
 * hasForced，以及 JPS3DNeib 的 Neib/FNeib 查表生成函数。
 */
#pragma once

#include <iostream>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <Eigen/Eigen>

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

/**
 * JPS 三维邻居查找表。
 * ns 保存自然邻居，f1 保存需要检查的强迫邻居位置，f2 保存障碍触发后真正扩展的方向。
 * 构造时一次性生成 27 种运动方向的规则，搜索过程中直接查表以减少分支判断。
 */
struct JPS3DNeib {
	// for each (dx,dy,dz) these contain:
	//    ns: neighbors that are always added
	//    f1: forced neighbors to check
	//    f2: neighbors to add if f1 is forced
	int ns[27][3][26];
	int f1[27][3][12];
	int f2[27][3][12];
	// nsz contains the number of neighbors for the four different types of moves:
	// no move (norm 0):        26 neighbors always added
	//                          0 forced neighbors to check (never happens)
	//                          0 neighbors to add if forced (never happens)
	// straight (norm 1):       1 neighbor always added
	//                          8 forced neighbors to check
	//                          8 neighbors to add if forced
	// diagonal (norm sqrt(2)): 3 neighbors always added
	//                          8 forced neighbors to check
	//                          12 neighbors to add if forced
	// diagonal (norm sqrt(3)): 7 neighbors always added
	//                          6 forced neighbors to check
	//                          12 neighbors to add if forced
	static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
	JPS3DNeib();  // 生成全部自然邻居与强迫邻居查找表。
	private:
	// 按运动维数和序号生成一个自然邻居方向。
	void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
	// 生成一对“障碍检查方向”和“障碍触发后的扩展方向”。
	void FNeib( int dx, int dy, int dz, int norm1, int dev,
	    int& fx, int& fy, int& fz,
	    int& nx, int& ny, int& nz);
};

/** 单个三维栅格节点，记录索引、坐标、搜索状态、代价和父节点。 */
struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index;
	
    bool is_path;
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    // 用给定索引和中心坐标创建未访问节点，并把 g/f 代价初始化为无穷大。
    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		is_path = false;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

/**
 * 三维栅格路径搜索器。
 * 同一个 graphSearch 接口可通过 use_jps 选择普通 A* 或 JPS；地图使用一维字节数组保存占据状态。
 */
class gridPathFinder
{
	private:
		// 三种启发函数；getHeu 统一选择实际使用的启发函数。
		double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
		double getManhHeu(GridNodePtr node1, GridNodePtr node2);
		double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
		double getHeu(GridNodePtr node1, GridNodePtr node2);

		// 从终点沿 cameFrom 回溯路径。
		std::vector<GridNodePtr> retrievePath(GridNodePtr current);

		double resolution, inv_resolution;
		double tie_breaker = 1.0 + 1.0 / 10000;

		std::vector<GridNodePtr> expandedNodes;
		std::vector<GridNodePtr> gridPath;
		std::vector<GridNodePtr> endPtrList;
		
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
	
		Eigen::Vector3i goalIdx;

		uint8_t * data;

		GridNodePtr *** GridNodeMap;
		std::multimap<double, GridNodePtr> openSet;
		JPS3DNeib * jn3d;

		// JPS 沿 expDir 递归跳跃，直到障碍、目标或跳点。
		bool jump(const Eigen::Vector3i & curIdx, const Eigen::Vector3i & expDir, Eigen::Vector3i & neiIdx);
		
		// 分别生成 JPS 剪枝后继和普通 26 邻域后继。
		inline void getJpsSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets, int num_iter);
		inline void getSucc   (GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
		// 判断当前位置沿给定方向运动时是否出现强迫邻居。
		inline bool hasForced(const Eigen::Vector3i & idx, const Eigen::Vector3i & dir);

		// 占据检查同时负责边界保护；越界不视为自由空间。
		inline bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		inline bool isOccupied(const Eigen::Vector3i & index) const;
		inline bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		inline bool isFree(const Eigen::Vector3i & index) const;

		// 世界坐标与栅格索引的双向转换。
		inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index) const;
		inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt) const;

	public:

		std::vector<Eigen::Vector3d> debugNodes;
		// 创建 JPS 邻居规则表；三维地图稍后由 initGridMap 分配。
		gridPathFinder( ){				
    		jn3d = new JPS3DNeib();
		};

		// 释放 JPS 邻居规则对象。
		~gridPathFinder(){
			delete jn3d;
		};

		// 分配三维节点表与占据数组，并设置地图边界、尺寸和分辨率。
		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		// 将一个世界坐标对应的栅格标记为障碍。
		void setObs(const double coord_x, const double coord_y, const double coord_z);

		// 执行一次 A* 或 JPS 搜索。
		void graphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_jps = false);
		// 清理单个节点或清理上一轮实际使用过的全部节点。
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		// 将任意坐标吸附到最近的栅格中心，并提供路径/访问节点查询接口。
		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord) const;
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
		std::vector<Eigen::Vector3d> getCloseNodes();
};
