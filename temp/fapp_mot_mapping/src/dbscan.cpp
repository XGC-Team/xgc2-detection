// 标准库头文件
#include <vector>

// DBSCAN聚类算法头文件
#include "dbscan.h"
// KD树数据结构头文件，用于高效的空间查询
#include "kdtree.h"
// 点数据结构头文件
#include "point.h"

/**
 * @brief DBSCAN聚类算法实现
 *
 * DBSCAN (Density-Based Spatial Clustering of Applications with Noise) 是一种基于密度的聚类算法
 * 该算法能够发现任意形状的簇，并能有效识别噪声点
 *
 * @param points 输入的点集，包含所有需要聚类的点
 * @param epsilon ε邻域半径，定义了两个点被认为是邻居的最大距离
 * @param minPoints 最小点数阈值，定义了形成一个簇所需的最少邻居点数（包括自身）
 * @return std::vector<std::vector<unsigned long>> 聚类结果，每个内层向量包含属于同一簇的点的索引
 *
 * 算法原理：
 * 1. 核心点：在ε邻域内至少有minPoints个点的点
 * 2. 边界点：在某个核心点的ε邻域内，但自身不是核心点
 * 3. 噪声点：既不是核心点也不是边界点
 * 4. 从核心点出发，将密度可达的点归为同一簇
 */
std::vector<std::vector<unsigned long>> dbscan::cluster(
    std::vector<Point>& points, const float& epsilon, const int& minPoints)
{
    // 调用kdtree的cluster方法执行实际的聚类操作
    // kdtree提供了高效的邻域查询，加速DBSCAN算法的执行
    std::vector<std::vector<unsigned long>> clusters
        = kdtree::cluster(points, epsilon, minPoints);

    // 返回聚类结果
    return clusters;
}
