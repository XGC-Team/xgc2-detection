#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "point.h"

// DBSCAN算法的原始实现
// 基于维基百科提供的算法
// 参考: https://en.wikipedia.org/wiki/DBSCAN
//
namespace original {

const int FAIL = -3;  // 失败状态码，表示搜索失败
const int PASS = 0;   // 成功状态码，表示搜索成功

/**
 * @brief 范围查询函数 - 查找给定点的邻域内的所有点
 * @param sptr_points 指向所有点集合的智能指针
 * @param core 核心点，用于查询邻域
 * @param epsilon 邻域半径阈值，距离小于等于此值的点被认为是邻居
 * @return 返回核心点邻域内所有点的索引列表
 */
std::vector<int> queryRange(std::shared_ptr<std::vector<Point>>& sptr_points,
    const Point& core, const float& epsilon)
{
    int index = 0;  // 当前点的索引
    std::vector<int> neighbours;  // 存储邻居点的索引
    // 遍历所有点，查找距离核心点epsilon范围内的点
    for (const auto& point : *sptr_points) {
        if (core.distance(point) <= epsilon) {
            neighbours.push_back(index);  // 将邻居点的索引加入列表
        }
        index++;
    }
    return neighbours;  // 返回邻居点索引列表
}

/**
 * @brief 搜索函数 - 基于密度的聚类扩展搜索
 * @param sptr_points 指向所有点集合的智能指针
 * @param point 当前要搜索的点
 * @param cluster 当前聚类的簇标签
 * @param N 最小邻域点数阈值（minPts），邻域内至少需要N个点才能成为核心点
 * @param E 邻域半径（epsilon）
 * @return 返回PASS表示成功找到一个簇，返回FAIL表示该点为噪声点
 */
int search(std::shared_ptr<std::vector<Point>>& sptr_points, Point point,
    int cluster, const int& N, const float& E)
{
    // 查询当前点的邻域
    std::vector<int> neighbours = queryRange(sptr_points, point, E);
    // 如果邻域内点数小于最小点数阈值，则该点为噪声点
    if (neighbours.size() < N) {
        point.m_cluster = NOISE;
        return FAIL;
    }

    int index = 0;  // 邻居点的索引
    int core = 0;   // 核心点在邻居列表中的位置
    // 将所有邻居点标记为当前簇，并找到核心点自身在邻居列表中的位置
    for (auto neighbour : neighbours) {
        sptr_points->at(neighbour).m_cluster = cluster;
        if (sptr_points->at(neighbour) == point) {
            core = index;
        }
        ++index;
    }
    // 从邻居列表中移除核心点自身（避免重复处理）
    neighbours.erase(neighbours.begin() + core);
    // 遍历所有邻居点，进行密度可达性扩展
    for (std::vector<int>::size_type i = 0, n = neighbours.size(); i < n; ++i) {
        // 查询邻居点的邻域
        std::vector<int> nextSet
            = queryRange(sptr_points, sptr_points->at(neighbours[i]), E);

        // 如果邻居点也是核心点（邻域内点数 >= N）
        if (nextSet.size() >= N) {
            // 将其邻域内未标记的点加入当前簇
            for (auto neighbour : nextSet) {
                if (sptr_points->at(neighbour).unlabeled()) {
                    neighbours.push_back(neighbour);  // 将新点加入待处理列表
                    n = neighbours.size();            // 更新列表大小
                    sptr_points->at(neighbour).m_cluster = cluster;  // 标记为当前簇
                }
            }
        }
    }
    return PASS;  // 成功完成聚类扩展
}

/**
 * @brief DBSCAN聚类主函数 - 对所有点进行基于密度的聚类
 * @param sptr_points 指向所有点集合的智能指针
 * @param N 最小邻域点数阈值（minPts），定义核心点的密度要求
 * @param E 邻域半径（epsilon），定义邻域的范围
 * @return 返回聚类的总数量（不包括噪声点）
 *
 * 算法流程：
 * 1. 遍历所有未标记的点
 * 2. 对每个未标记点执行密度搜索
 * 3. 如果该点能成为核心点，则形成一个新簇，簇标签递增
 * 4. 如果该点不能成为核心点，则标记为噪声点
 */
int cluster(std::shared_ptr<std::vector<Point>>& sptr_points, const int& N,
    const float& E)
{
    int cluster = 0;  // 簇计数器，从0开始递增
    // 遍历所有点
    for (const auto& point : *sptr_points) {
        // 只处理未标记的点
        if (point.m_cluster == UNLABELED) {
            // 尝试以该点为起点进行密度扩展搜索
            if (search(sptr_points, point, cluster, N, E) != FAIL) {
                cluster += 1;  // 成功找到一个新簇，簇标签递增
            }
            // 如果搜索失败，该点已被标记为噪声点
        }
    }
    return cluster;  // 返回找到的簇总数
}
}  // namespace original
