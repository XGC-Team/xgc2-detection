/*
    FILE: dbscan.h
    ------------------
    DBSCAN聚类算法的辅助类函数定义
    DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
    是一种基于密度的空间聚类算法，能够发现任意形状的簇并识别噪声点
*/
#include <onboard_detector_lv/dbscan.h>
#include <iostream>

namespace onboardDetector{
    /**
     * @brief DBSCAN聚类算法主函数
     * @return 返回0表示执行成功
     *
     * 功能：遍历所有未分类的点，尝试以每个点为核心扩展聚类
     * 算法流程：
     * 1. 初始化聚类ID为1
     * 2. 遍历所有点，对未分类的点尝试扩展聚类
     * 3. 成功扩展一个聚类后，聚类ID递增
     */
    int DBSCAN::run()
    {
        int clusterID = 1;  // 聚类ID初始化为1，0通常保留给未分类状态
        vector<Point>::iterator iter;
        // 遍历所有点
        for(iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            // 只处理未分类的点（UNCLASSIFIED状态）
            if ( iter->clusterID == UNCLASSIFIED )
            {
                // 尝试以当前点为核心扩展聚类
                if ( expandCluster(*iter, clusterID) != FAILURE )
                {
                    clusterID += 1;  // 成功扩展一个聚类后，聚类ID递增，准备下一个聚类
                }
            }
        }

        return 0;
    }

    /**
     * @brief 扩展聚类函数 - DBSCAN算法的核心
     * @param point 待扩展的核心点
     * @param clusterID 当前聚类的ID
     * @return SUCCESS表示成功扩展聚类，FAILURE表示该点为噪声点
     *
     * 功能：从一个核心点开始，基于密度可达性扩展聚类
     * 算法流程：
     * 1. 计算核心点的epsilon邻域内的所有点
     * 2. 如果邻域点数小于minPoints，标记为噪声点
     * 3. 否则，将邻域内所有点标记为当前聚类
     * 4. 递归地扩展聚类，检查每个邻域点是否也是核心点
     * 5. 如果是核心点，将其邻域内的未分类点和噪声点加入当前聚类
     */
    int DBSCAN::expandCluster(Point point, int clusterID)
    {
        // 计算当前点的epsilon邻域，获取所有在epsilon距离内的点的索引
        vector<int> clusterSeeds = calculateCluster(point);

        // 如果邻域点数小于minPoints，该点不是核心点，标记为噪声
        if ( clusterSeeds.size() < m_minPoints )
        {
            point.clusterID = NOISE;  // 标记为噪声点
            return FAILURE;  // 返回失败，不能形成聚类
        }
        else  // 邻域点数 >= minPoints，该点是核心点，可以形成聚类
        {
            int index = 0, indexCorePoint = 0;
            vector<int>::iterator iterSeeds;
            // 将邻域内的所有点都标记为当前聚类ID
            for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
            {
                m_points.at(*iterSeeds).clusterID = clusterID;  // 分配聚类ID
                // 找到核心点本身在seeds中的位置
                if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
                {
                    indexCorePoint = index;  // 记录核心点的索引
                }
                ++index;
            }
            // 从seeds中移除核心点本身，因为我们要处理的是它的邻域点
            clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

            // 迭代扩展聚类：检查每个邻域点是否也是核心点
            // 注意：这里使用索引而不是迭代器，因为clusterSeeds会动态增长
            for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
            {
                // 计算当前邻域点的邻域
                vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

                // 如果该邻域点也是核心点（邻域点数 >= minPoints）
                if ( clusterNeighors.size() >= m_minPoints )
                {
                    vector<int>::iterator iterNeighors;
                    // 遍历该核心点的所有邻域点
                    for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                    {
                        // 只处理未分类点或噪声点
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                        {
                            // 如果是未分类点，加入seeds队列以便后续处理
                            if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                            {
                                clusterSeeds.push_back(*iterNeighors);  // 加入待处理队列
                                n = clusterSeeds.size();  // 更新队列大小
                            }
                            // 将该点标记为当前聚类（包括之前的噪声点）
                            m_points.at(*iterNeighors).clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;  // 成功扩展聚类
        }
    }

    /**
     * @brief 计算给定点的epsilon邻域
     * @param point 查询点（核心点）
     * @return 返回所有在epsilon距离内的点的索引向量
     *
     * 功能：找出距离给定点不超过epsilon的所有点
     * 这是DBSCAN算法中判断密度可达性的基础
     */
    vector<int> DBSCAN::calculateCluster(Point point)
    {
        int index = 0;  // 点的索引计数器
        vector<Point>::iterator iter;
        vector<int> clusterIndex;  // 存储邻域内点的索引
        // 遍历所有点
        for( iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            // 计算距离，如果在epsilon范围内，则加入邻域
            if ( calculateDistance(point, *iter) <= m_epsilon )
            {
                clusterIndex.push_back(index);  // 记录该点的索引
            }
            index++;
        }
        return clusterIndex;  // 返回邻域内所有点的索引
    }

    /**
     * @brief 计算两点之间的欧几里得距离的平方
     * @param pointCore 核心点
     * @param pointTarget 目标点
     * @return 返回两点间的欧几里得距离的平方
     *
     * 注意：这里返回的是距离的平方而不是距离本身
     * 这样可以避免开方运算，提高计算效率
     * 因此在使用时，m_epsilon应该是距离阈值的平方
     */
    inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
    {
        // 计算三维空间中两点距离的平方：(x1-x2)² + (y1-y2)² + (z1-z2)²
        return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
    }
}


