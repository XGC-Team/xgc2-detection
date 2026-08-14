// DBSCAN聚类算法头文件
// DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
// 基于密度的空间聚类算法，用于发现任意形状的聚类并识别噪声点
#ifndef DBSCAN_H
#define DBSCAN_H


#include <vector>

// dbscan命名空间 - 封装DBSCAN聚类算法接口
namespace dbscan {

/**
 * @brief DBSCAN聚类函数 - 执行基于密度的空间聚类
 *
 * DBSCAN是一种基于密度的聚类算法，能够：
 * 1. 发现任意形状的聚类
 * 2. 自动识别噪声点（不属于任何聚类的点）
 * 3. 不需要预先指定聚类数量
 *
 * 算法原理：
 * - 核心点：在epsilon半径内至少有minPoints个邻居点
 * - 边界点：在epsilon半径内邻居点少于minPoints，但在核心点的邻域内
 * - 噪声点：既不是核心点也不是边界点
 *
 * @param points 未聚类的点集合
 *   输入的原始点云数据，每个点包含3D坐标信息
 *
 * @param epsilon ε参数（邻域半径）
 *   定义点的邻域范围，单位通常为米
 *   较小的epsilon会产生更多、更小的聚类
 *   较大的epsilon会产生更少、更大的聚类
 *
 * @param minPoints 最小点数阈值
 *   一个点成为核心点所需的最小邻居数量（包括该点本身）
 *   较小的minPoints会产生更多聚类
 *   较大的minPoints会产生更少、更密集的聚类
 *
 * @return 聚类结果的索引集合
 *   返回一个二维向量，外层向量的每个元素代表一个聚类
 *   内层向量包含属于该聚类的点在原始points向量中的索引
 *   例如：{{0,1,5}, {2,3,4}, {6,7}} 表示3个聚类
 */
std::vector<std::vector<unsigned long>> cluster(
    std::vector<Point>& points, const float& epsilon, const int& minPoints);
}
#endif /* DBSCAN_H */
