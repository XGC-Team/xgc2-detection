/*
    FILE: lidarDetector.h
    ---------------------------------
    header file of lidar-based obstacle detector

    功能说明：
    基于激光雷达的障碍物检测器头文件，实现点云聚类和障碍物检测功能
*/
#ifndef ONBOARDDETECTOR_LIDARDETECTOR_H
#define ONBOARDDETECTOR_LIDARDETECTOR_H

#include <ros/ros.h>
#include <onboard_detector_lv/dbscan.h>
#include <onboard_detector_lv/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
namespace onboardDetector{
    /**
     * @struct Cluster
     * @brief 聚类结构体，存储单个聚类的所有信息
     *
     * 该结构体用于表示激光雷达点云聚类的结果，包含聚类的几何信息、
     * 主成分分析(PCA)结果以及原始点云数据
     */
    struct Cluster
    {
        int cluster_id;               // 聚类ID，用于唯一标识该聚类
        Eigen::Vector4f centroid;     // 聚类质心，4维向量(x,y,z,1)
        pcl::PointCloud<pcl::PointXYZ>::Ptr points; // 聚类点云数据，智能指针管理

        // 几何信息
        Eigen::Vector3f dimensions;    // 边界框尺寸(长、宽、高)
        Eigen::Matrix3f eigen_vectors; // PCA特征向量矩阵，表示主方向
        Eigen::Vector3f eigen_values;  // PCA特征值向量，表示各主方向的方差

        /**
         * @brief 默认构造函数，初始化聚类数据结构
         *
         * 将聚类ID设为-1(未分配)，质心设为零向量，
         * 点云指针初始化为新的空点云对象
         */
        Cluster():
            cluster_id(-1),
            centroid(Eigen::Vector4f::Zero()),
            points(new pcl::PointCloud<pcl::PointXYZ>()) {}
    };

    /**
     * @class lidarDetector
     * @brief 激光雷达障碍物检测器类
     *
     * 基于DBSCAN聚类算法实现激光雷达点云的障碍物检测功能，
     * 提供点云输入、聚类处理、边界框提取等核心功能
     */
    class lidarDetector{
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // 当前帧点云数据，存储待处理的激光雷达扫描点
        std::vector<onboardDetector::Cluster> clusters_; // 当前帧聚类结果列表，存储所有检测到的障碍物聚类
        std::vector<onboardDetector::box3D> bboxes_; // 激光雷达边界框列表，存储每个聚类的3D包围盒

        // 激光雷达DBSCAN聚类参数
        double eps_;          // DBSCAN邻域半径(epsilon)，定义点的邻域范围(单位:米)
        int minPts_;          // DBSCAN最小点数，形成核心点所需的最小邻域点数
        double groundHeight_; // 地面高度阈值，用于滤除地面点(单位:米)
        double roofHeight_;   // 顶部高度阈值，用于滤除过高的点(单位:米)
        
    public:
        /**
         * @brief 默认构造函数
         *
         * 初始化激光雷达检测器，创建空的点云和聚类容器
         */
        lidarDetector();

        /**
         * @brief 设置DBSCAN聚类算法参数
         *
         * @param eps DBSCAN邻域半径参数(单位:米)，决定点之间的邻域距离阈值
         * @param minPts DBSCAN最小点数参数，决定形成聚类所需的最小邻域点数
         *
         * 说明：
         * - eps越大，聚类越宽松，更多点会被归入同一聚类
         * - minPts越大，形成聚类的要求越严格，可以过滤噪声点
         */
        void setParams(double eps, int minPts);

        /**
         * @brief 接收并存储输入点云数据
         *
         * @param cloud 输入点云的智能指针，包含激光雷达扫描的三维点数据
         *
         * 说明：该函数将外部点云数据拷贝到检测器内部存储，为后续聚类处理做准备
         */
        void getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        /**
         * @brief 执行基于激光雷达点云的DBSCAN聚类
         *
         * 功能说明：
         * 1. 对当前存储的点云执行DBSCAN密度聚类算法
         * 2. 将点云分割成多个障碍物聚类
         * 3. 计算每个聚类的质心、边界框等几何信息
         * 4. 执行PCA主成分分析，提取障碍物的主方向
         *
         * 注意：调用前需确保已通过getPointcloud()设置了有效的点云数据
         */
        void lidarDBSCAN();

        /**
         * @brief 获取当前帧的聚类结果
         *
         * @return 聚类结果向量的引用，包含所有检测到的障碍物聚类
         *
         * 说明：返回引用可避免数据拷贝，提高效率，但需注意生命周期管理
         */
        std::vector<onboardDetector::Cluster>& getClusters();

        /**
         * @brief 获取当前帧的三维边界框列表
         *
         * @return 3D边界框向量的引用，每个边界框对应一个检测到的障碍物
         *
         * 说明：边界框包含障碍物的位置、尺寸和方向信息，用于碰撞检测和路径规划
         */
        std::vector<onboardDetector::box3D>& getBBoxes();
    };
}


#endif
