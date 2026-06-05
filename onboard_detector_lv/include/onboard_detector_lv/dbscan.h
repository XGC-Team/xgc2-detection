/*
    FILE: dbscan.h
    ------------------
    DBSCAN (Density-Based Spatial Clustering of Applications with Noise) 聚类算法辅助类头文件

    DBSCAN是一种基于密度的聚类算法，主要用于：
    - 从包含噪声的空间数据中识别任意形状的聚类
    - 自动确定聚类数量
    - 能够识别并排除噪声点

    算法核心思想：
    - 核心点(Core Point): 在半径eps内至少有minPts个点
    - 边界点(Border Point): 在某核心点的邻域内，但自身不是核心点
    - 噪声点(Noise): 既不是核心点也不是边界点
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

// 点的分类状态定义
#define UNCLASSIFIED -1    // 未分类状态：点尚未被处理
#define CORE_POINT 1       // 核心点：邻域内点数 >= minPts
#define BORDER_POINT 2     // 边界点：在核心点邻域内但自身不是核心点
#define NOISE -2           // 噪声点：既不是核心点也不是边界点

// 算法执行状态定义
#define SUCCESS 0          // 成功执行
#define FAILURE -3         // 执行失败

using namespace std;
namespace onboardDetector{
    /**
     * @brief 三维空间点结构体
     *
     * 用于表示DBSCAN算法中的数据点，包含空间坐标和聚类信息
     */
    typedef struct Point_
    {
        float x, y, z;  // 三维空间坐标：X、Y、Z位置
        int clusterID;  // 聚类ID：标识该点所属的聚类编号，初始为UNCLASSIFIED
    }Point;

    /**
     * @brief DBSCAN聚类算法类
     *
     * 实现基于密度的空间聚类算法，适用于三维点云数据的聚类分析
     *
     * 算法流程：
     * 1. 遍历所有未访问的点
     * 2. 对每个点，计算其邻域内的点数
     * 3. 如果邻域内点数 >= minPts，则为核心点，开始扩展聚类
     * 4. 递归地将邻域内的点添加到当前聚类
     * 5. 无法归入任何聚类的点标记为噪声
     */
    class DBSCAN {
    public:
        /**
         * @brief 构造函数
         *
         * @param minPts 最小点数：定义核心点的邻域最小点数阈值
         * @param eps 邻域半径(epsilon)：定义两点是否为邻居的距离阈值
         * @param points 待聚类的点集：包含所有需要进行聚类分析的三维点
         */
        DBSCAN(unsigned int minPts, float eps, vector<Point> points){
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();
        }

        /**
         * @brief 析构函数
         */
        ~DBSCAN(){}

        /**
         * @brief 执行DBSCAN聚类算法
         *
         * 主函数，对所有点进行聚类分析
         * 遍历所有点，识别核心点并扩展聚类，标记噪声点
         *
         * @return 返回SUCCESS(0)表示成功，FAILURE(-3)表示失败
         */
        int run();

        /**
         * @brief 计算指定点的邻域聚类
         *
         * 查找给定点eps邻域内的所有点，返回这些邻居点的索引列表
         * 用于判断该点是否为核心点，以及扩展聚类时查找邻域点
         *
         * @param point 目标点：需要计算邻域的点
         * @return 返回邻域内所有点的索引向量
         */
        vector<int> calculateCluster(Point point);

        /**
         * @brief 扩展聚类
         *
         * 从一个核心点开始，递归地将其邻域内的点添加到同一聚类
         * 如果邻域内的点也是核心点，则继续扩展其邻域
         * 这是DBSCAN算法的核心步骤，实现密度可达性的传播
         *
         * @param point 起始核心点：从该点开始扩展聚类
         * @param clusterID 聚类ID：当前聚类的标识号
         * @return 返回SUCCESS(0)表示成功扩展，FAILURE(-3)表示失败
         */
        int expandCluster(Point point, int clusterID);

        /**
         * @brief 计算两点之间的欧氏距离
         *
         * 计算三维空间中两点的欧氏距离，用于判断两点是否在邻域内
         * 距离公式: d = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
         *
         * @param pointCore 核心点：第一个点
         * @param pointTarget 目标点：第二个点
         * @return 返回两点之间的欧氏距离
         */
        inline double calculateDistance(const Point& pointCore, const Point& pointTarget);

        /**
         * @brief 获取点集总数
         *
         * @return 返回待聚类的点的总数量
         */
        int getTotalPointSize() {return m_pointSize;}

        /**
         * @brief 获取最小聚类规模参数
         *
         * @return 返回定义核心点的最小邻域点数(minPts)
         */
        int getMinimumClusterSize() {return m_minPoints;}

        /**
         * @brief 获取邻域半径参数
         *
         * @return 返回定义邻域的距离阈值(epsilon)
         */
        int getEpsilonSize() {return m_epsilon;}
        
    public:
        // 点集数据（公有成员，允许外部访问聚类结果）
        vector<Point> m_points;  // 所有待聚类的点，聚类后每个点的clusterID会被更新

    private:
        // 私有成员变量
        unsigned int m_pointSize;  // 点集中点的总数量
        unsigned int m_minPoints;  // 最小点数阈值(minPts)：核心点邻域内必须包含的最小点数
        float m_epsilon;           // 邻域半径(ε)：定义两点为邻居的最大距离阈值
    };
}
#endif // DBSCAN_H
