/**
 * @file DBSCAN_kdtree.h
 * @brief 基于KD树的DBSCAN聚类算法实现
 *
 * 本文件实现了使用PCL库中KD树进行邻域搜索的DBSCAN聚类算法。
 * 相比于DBSCAN_simple.h中的基础实现，此版本利用KD树加速半径搜索，
 * 在大规模点云数据上具有更好的性能表现。
 */

#ifndef DBSCAN_KDTREE_H
#define DBSCAN_KDTREE_H

#include <pcl/point_types.h>
#include "DBSCAN_simple.h"

/**
 * @class DBSCANKdtreeCluster
 * @brief 使用KD树优化的DBSCAN聚类器
 *
 * 该类继承自DBSCANSimpleCluster，重写了半径搜索方法，
 * 使用PCL库提供的KD树数据结构进行快速邻域搜索。
 *
 * DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
 * 是一种基于密度的聚类算法，能够发现任意形状的簇并识别噪声点。
 *
 * @tparam PointT PCL点类型（如pcl::PointXYZ, pcl::PointXYZI等）
 */
template <typename PointT>
class DBSCANKdtreeCluster: public DBSCANSimpleCluster<PointT> {
protected:
    /**
     * @brief 执行半径搜索以查找邻近点
     *
     * 此方法重写了基类中的虚函数，使用PCL的KD树实现高效的半径搜索。
     * KD树的时间复杂度为O(log n)，相比暴力搜索的O(n)有显著性能提升。
     *
     * @param index 查询点在点云中的索引
     * @param radius 搜索半径（对应DBSCAN算法中的eps参数）
     * @param k_indices 输出参数：存储在搜索半径内找到的点的索引
     * @param k_sqr_distances 输出参数：存储对应点到查询点的平方距离
     * @return 找到的邻近点数量
     */
    virtual int radiusSearch (
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        // 调用PCL KD树的半径搜索方法
        // search_method_是从基类继承的KD树智能指针
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }

}; // class DBSCANKdtreeCluster

#endif // DBSCAN_KDTREE_H