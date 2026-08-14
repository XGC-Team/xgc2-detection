/*
    FILE: lidarDetector.cpp
    ------------------
    基于激光雷达的障碍物检测器类函数定义
    功能：使用DBSCAN聚类算法对激光雷达点云数据进行障碍物检测和聚类
*/
#include <onboard_detector_lv/lidarDetector.h>
namespace onboardDetector{
    /**
     * @brief 激光雷达检测器构造函数
     * @details 初始化DBSCAN聚类算法的默认参数和点云指针
     */
    lidarDetector::lidarDetector(){
        this->eps_ = 0.5;          // DBSCAN邻域半径，单位：米
        this->minPts_ = 10;        // DBSCAN最小点数阈值，定义核心点的最小邻域点数
        this->cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());  // 初始化点云智能指针
    }

    /**
     * @brief 设置DBSCAN聚类算法参数
     * @param eps DBSCAN邻域半径（epsilon），定义点之间的最大距离阈值
     * @param minPts DBSCAN最小点数，一个点成为核心点所需的最小邻域点数
     */
    void lidarDetector::setParams(double eps, int minPts){
        this->eps_ = eps;
        this->minPts_ = minPts;
    }

    /**
     * @brief 接收并存储输入的点云数据
     * @param cloud 输入的PCL点云数据指针（包含XYZ坐标信息）
     */
    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;  // 保存点云数据的共享指针
    }

    /**
     * @brief 使用DBSCAN算法对激光雷达点云进行聚类分析
     * @details 执行完整的点云聚类流程：
     *          1. 点云格式转换
     *          2. DBSCAN聚类计算
     *          3. 提取聚类结果
     *          4. 计算每个聚类的质心和包围盒
     */
    void lidarDetector::lidarDBSCAN(){
        // 检查点云是否有效且非空
        if(!cloud_ || cloud_->empty()){
            return;
        }

        // 将PCL点云格式转换为DBSCAN算法所需的Point格式
        std::vector<Point> points;
        for(size_t i=0; i<cloud_->size(); ++i){
            Point p;
            p.x = cloud_->points[i].x;           // 提取X坐标
            p.y = cloud_->points[i].y;           // 提取Y坐标
            p.z = cloud_->points[i].z;           // 提取Z坐标
            p.clusterID = UNCLASSIFIED;          // 初始化聚类ID为未分类状态
            points.push_back(p);
        }

        // 创建DBSCAN对象并执行聚类算法
        DBSCAN dbscan(minPts_, eps_, points);
        dbscan.run();  // 运行DBSCAN聚类，为每个点分配聚类ID


        // 统计聚类数量：遍历所有点找出最大的聚类ID
        int clusterNum = 0;
        for (size_t i=0; i<dbscan.m_points.size(); ++i){
            onboardDetector::Point pDB = dbscan.m_points[i];
            if (pDB.clusterID > clusterNum){
                clusterNum = pDB.clusterID;  // 更新最大聚类ID（聚类总数）
            }
        }


        // 根据聚类数量初始化聚类容器
        std::vector<onboardDetector::Cluster> clustersTemp;
        clustersTemp.resize(clusterNum);  // 预分配内存空间

        // 将聚类后的点分配到对应的Cluster对象中
        for(size_t i=0; i<dbscan.m_points.size(); ++i){
            if (dbscan.m_points[i].clusterID > 0){  // 过滤噪声点（clusterID <= 0为噪声）
                pcl::PointXYZ point;
                point.x = dbscan.m_points[i].x;
                point.y = dbscan.m_points[i].y;
                point.z = dbscan.m_points[i].z;
                // 将点添加到对应聚类的点云中（clusterID从1开始，数组索引从0开始）
                clustersTemp[dbscan.m_points[i].clusterID-1].points->push_back(point);
                clustersTemp[dbscan.m_points[i].clusterID-1].cluster_id = dbscan.m_points[i].clusterID;
            }
        }
        this->clusters_ = clustersTemp;  // 保存聚类结果

        // 为每个聚类计算3D包围盒（bounding box）
        std::vector<onboardDetector::box3D> bboxesTemp;
        for(auto& cluster : this->clusters_){
            // 计算聚类的质心（centroid）：所有点的平均位置
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster.points, centroid);  // PCL库函数计算3D质心
            cluster.centroid = centroid;

            // 获取聚类点云的最小和最大边界点
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cluster.points, minPt, maxPt);  // 找出XYZ各轴的最小最大值

            // 计算包围盒的尺寸（长宽高）
            cluster.dimensions = Eigen::Vector3f(maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z);

            // 构建3D包围盒对象
            onboardDetector::box3D bbox;
            bbox.x = centroid(0);                // 包围盒中心X坐标
            bbox.y = centroid(1);                // 包围盒中心Y坐标
            bbox.z = centroid(2);                // 包围盒中心Z坐标
            bbox.x_width = maxPt.x - minPt.x;    // X方向宽度
            bbox.y_width = maxPt.y - minPt.y;    // Y方向宽度
            bbox.z_width = maxPt.z - minPt.z;    // Z方向高度
            bboxesTemp.push_back(bbox);
        }
        this->bboxes_ = bboxesTemp;  // 保存所有包围盒结果
    }

    /**
     * @brief 获取聚类结果
     * @return 返回所有聚类对象的引用，每个聚类包含点云、质心和尺寸信息
     */
    std::vector<onboardDetector::Cluster>& lidarDetector::getClusters(){
        return this->clusters_;
    }

    /**
     * @brief 获取3D包围盒结果
     * @return 返回所有障碍物的3D包围盒引用，包含中心位置和尺寸信息
     */
    std::vector<onboardDetector::box3D>& lidarDetector::getBBoxes(){
        return this->bboxes_;
    }
}  // namespace onboardDetector
