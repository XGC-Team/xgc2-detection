/*
    FILE: dynamicDetector.h
    ---------------------------------
    动态障碍物检测器头文件
    功能：融合视觉(深度相机)和激光雷达数据进行动态障碍物检测、跟踪和分类
    主要技术：DBSCAN聚类、卡尔曼滤波跟踪、数据关联、YOLO目标检测融合
*/
#ifndef ONBOARDDETECTOR_DYNAMICDETECTOR_H
#define ONBOARDDETECTOR_DYNAMICDETECTOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <onboard_detector_lv/dbscan.h>
#include <onboard_detector_lv/uvDetector.h>
#include <onboard_detector_lv/lidarDetector.h>
#include <onboard_detector_lv/kalmanFilter.h>
#include <onboard_detector_lv/utils.h>
#include <onboard_detector_lv/GetDynamicObstacles.h>

namespace onboardDetector{
    /**
     * @class dynamicDetector
     * @brief 动态障碍物检测器类
     *
     * 该类实现了基于激光雷达-视觉(LiDAR-Vision)融合的动态障碍物检测系统
     * 主要功能包括:
     * 1. 深度图像处理和3D点云投影
     * 2. 激光雷达点云处理
     * 3. DBSCAN聚类检测
     * 4. 多传感器数据融合
     * 5. 卡尔曼滤波目标跟踪
     * 6. 动静态分类
     * 7. YOLO检测结果融合
     */
    class dynamicDetector{
    private:
        // ========== 基本配置 ==========
        std::string ns_;    // 命名空间
        std::string hint_;  // 提示信息

        // ========== ROS通信接口 ==========
        ros::NodeHandle nh_;  // ROS节点句柄

        // 订阅器 - 使用消息同步机制
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;  // 深度图像订阅器
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidarCloudSub_;  // 激光雷达点云订阅器
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;  // 位姿订阅器

        // 深度图像与位姿同步策略
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> depthPoseSync;
        std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;  // 深度图-位姿同步器

        // 激光雷达点云与位姿同步策略
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> lidarPoseSync;
        std::shared_ptr<message_filters::Synchronizer<lidarPoseSync>> lidarPoseSync_;  // 雷达-位姿同步器

        // 里程计相关订阅器
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;  // 里程计订阅器

        // 深度图像与里程计同步策略
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> depthOdomSync;
        std::shared_ptr<message_filters::Synchronizer<depthOdomSync>> depthOdomSync_;  // 深度图-里程计同步器

        // 激光雷达与里程计同步策略
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> lidarOdomSync;
        std::shared_ptr<message_filters::Synchronizer<lidarOdomSync>> lidarOdomSync_;  // 雷达-里程计同步器

        // 普通订阅器
        ros::Subscriber colorImgSub_;  // 彩色图像订阅器
        ros::Subscriber yoloDetectionSub_;  // YOLO检测结果订阅器

        // 定时器 - 控制各个处理流程的执行频率
        ros::Timer detectionTimer_;  // 视觉检测定时器
        ros::Timer lidarDetectionTimer_;  // 激光雷达检测定时器
        ros::Timer trackingTimer_;  // 跟踪定时器
        ros::Timer classificationTimer_;  // 分类定时器
        ros::Timer visTimer_;  // 可视化定时器

        // 图像发布器
        image_transport::Publisher uvDepthMapPub_;  // UV深度图发布器
        image_transport::Publisher uDepthMapPub_;  // U深度图发布器
        image_transport::Publisher uvBirdViewPub_;  // UV鸟瞰图发布器
        image_transport::Publisher detectedColorImgPub_;  // 检测结果彩色图发布器

        // 边界框发布器
        ros::Publisher uvBBoxesPub_;  // UV检测边界框发布器
        ros::Publisher dbBBoxesPub_;  // DBSCAN聚类边界框发布器
        ros::Publisher visualBBoxesPub_;  // 视觉检测边界框发布器
        ros::Publisher lidarBBoxesPub_;  // 激光雷达检测边界框发布器
        ros::Publisher filteredBBoxesBeforeYoloPub_;  // YOLO融合前的过滤边界框发布器
        ros::Publisher filteredBBoxesPub_;  // 过滤后的边界框发布器
        ros::Publisher trackedBBoxesPub_;  // 跟踪边界框发布器
        ros::Publisher dynamicBBoxesPub_;  // 动态障碍物边界框发布器

        // 点云发布器
        ros::Publisher filteredDepthPointsPub_;  // 过滤后的深度点云发布器
        ros::Publisher lidarClustersPub_;  // 激光雷达聚类发布器
        ros::Publisher filteredPointsPub_;  // 过滤点云发布器
        ros::Publisher dynamicPointsPub_;  // 动态点云发布器
        ros::Publisher rawDynamicPointsPub_;  // 原始动态点云发布器
        ros::Publisher downSamplePointsPub_;  // 降采样点云发布器
        ros::Publisher rawLidarPointsPub_;  // 原始激光雷达点云发布器

        // 轨迹和速度可视化发布器
        ros::Publisher historyTrajPub_;  // 历史轨迹发布器
        ros::Publisher velVisPub_;  // 速度可视化发布器

        // 服务器
        ros::ServiceServer getDynamicObstacleServer_;  // 获取动态障碍物服务
    
        // ========== 检测器模块 ==========
        std::shared_ptr<onboardDetector::UVdetector> uvDetector_;  // UV(图像平面)检测器
        std::shared_ptr<onboardDetector::DBSCAN> dbCluster_;  // DBSCAN聚类器
        std::shared_ptr<onboardDetector::lidarDetector> lidarDetector_;  // 激光雷达检测器

        // ========== 传感器信息 ==========
        // 深度相机参数
        double fx_, fy_, cx_, cy_;  // 深度相机内参: 焦距(fx, fy)和主点坐标(cx, cy)
        double depthScale_;  // 深度缩放因子: 实际深度 = 像素值 / depthScale
        double depthMinValue_, depthMaxValue_;  // 深度值有效范围: 最小值和最大值
        double raycastMaxLength_;  // 光线投射最大长度
        int depthFilterMargin_, skipPixel_;  // 深度滤波边缘裕度、跳过的像素数(降采样)
        int imgCols_, imgRows_;  // 图像尺寸: 列数、行数
        Eigen::Matrix4d body2CamDepth_;  // 机体坐标系到深度相机坐标系的变换矩阵(4x4齐次变换)

        // 彩色相机参数
        double fxC_, fyC_, cxC_, cyC_;  // 彩色相机内参: 焦距(fx, fy)和主点坐标(cx, cy)
        Eigen::Matrix4d body2CamColor_;  // 机体坐标系到彩色相机坐标系的变换矩阵

        // 激光雷达参数
        Eigen::Matrix4d body2Lidar_;  // 机体坐标系到激光雷达坐标系的变换矩阵

        // ========== 系统参数配置 ==========
        // 话题名称配置
        int localizationMode_;  // 定位模式: 选择使用位姿(Pose)或里程计(Odom)
        std::string depthTopicName_;  // 深度图像话题名称
        std::string colorImgTopicName_;  // 彩色图像话题名称
        std::string lidarTopicName_;  // 激光雷达点云话题名称
        std::string poseTopicName_;  // 位姿话题名称
        std::string odomTopicName_;  // 里程计话题名称

        // 系统时间参数
        double dt_;  // 系统时间步长(秒), 用于运动预测和滤波

        // DBSCAN聚类公共参数
        double groundHeight_;  // 地面高度阈值, 用于过滤地面点
        double roofHeight_;  // 天花板高度阈值, 用于过滤上方点

        // DBSCAN视觉检测参数
        double voxelOccThresh_;  // 体素占用阈值, 判断体素是否被占据
        int dbMinPointsCluster_;  // DBSCAN最小聚类点数, 小于此值的聚类被视为噪声
        double dbEpsilon_;  // DBSCAN邻域半径(米), 定义点的邻域范围

        // DBSCAN激光雷达检测参数
        int lidarDBMinPoints_;  // 激光雷达DBSCAN最小聚类点数
        double lidarDBEpsilon_;  // 激光雷达DBSCAN邻域半径(米)
        int gaussianDownSampleRate_;  // 高斯降采样率
        int downSampleThresh_;  // 降采样阈值

        // 激光雷达-视觉融合过滤参数
        double boxIOUThresh_;  // 边界框IOU(交并比)阈值, 用于融合判断

        // 跟踪和数据关联参数
        double maxMatchRange_;  // 最大匹配距离(米), 超过此距离不进行匹配
        double maxMatchSizeRange_;  // 最大尺寸匹配范围, 尺寸差异阈值
        Eigen::VectorXd featureWeights_;  // 特征权重向量, 用于数据关联的特征加权
        int histSize_;  // 历史记录大小, 保存的历史帧数
        int fixSizeHistThresh_;  // 固定尺寸历史阈值, 判断目标尺寸是否稳定
        double fixSizeDimThresh_;  // 固定尺寸维度阈值, 尺寸稳定性判断

        // 卡尔曼滤波参数
        double eP_;  // 卡尔曼滤波初始不确定性矩阵参数
        double eQPos_;  // 运动模型位置不确定性矩阵参数
        double eQVel_;  // 运动模型速度不确定性矩阵参数
        double eQAcc_;  // 运动模型加速度不确定性矩阵参数
        double eRPos_;  // 观测模型位置不确定性矩阵参数
        double eRVel_;  // 观测模型速度不确定性矩阵参数
        double eRAcc_;  // 观测模型加速度不确定性矩阵参数
        int kfAvgFrames_;  // 卡尔曼滤波平均帧数, 用于速度估计

        // 动静态分类参数
        int skipFrame_;  // 跳过帧数, 分类前需要累积的帧数
        double dynaVelThresh_;  // 动态速度阈值(米/秒), 高于此值判定为动态
        double dynaVoteThresh_;  // 动态投票阈值, 投票率超过此值判定为动态
        int forceDynaFrames_;  // 强制动态帧数, 初始帧强制判定为动态
        int forceDynaCheckRange_;  // 强制动态检查范围, 检查的历史帧数
        int dynamicConsistThresh_;  // 动态一致性阈值, 连续判定为动态的帧数

        // 尺寸约束参数
        bool constrainSize_;  // 是否启用尺寸约束
        std::vector<Eigen::Vector3d> targetObjectSize_;  // 目标物体尺寸列表(长x宽x高)
        Eigen::Vector3d maxObjectSize_;  // 最大物体尺寸(长x宽x高), 超过此尺寸的检测被过滤 

        // ========== 传感器数据 ==========
        // 图像数据
        cv::Mat depthImage_;  // 深度图像(OpenCV格式)

        // 机器人位姿
        Eigen::Vector3d position_;  // 机器人位置(世界坐标系)
        Eigen::Matrix3d orientation_;  // 机器人姿态旋转矩阵(世界坐标系)

        // 深度相机位姿
        Eigen::Vector3d positionDepth_;  // 深度相机位置(世界坐标系)
        Eigen::Matrix3d orientationDepth_;  // 深度相机姿态旋转矩阵(世界坐标系)

        // 彩色相机位姿
        Eigen::Vector3d positionColor_;  // 彩色相机位置(世界坐标系)
        Eigen::Matrix3d orientationColor_;  // 彩色相机姿态旋转矩阵(世界坐标系)

        // 激光雷达位姿
        Eigen::Vector3d positionLidar_;  // 激光雷达位置(世界坐标系)
        Eigen::Matrix3d orientationLidar_;  // 激光雷达姿态旋转矩阵(世界坐标系)

        // 位姿状态标志
        bool hasSensorPose_;  // 是否已接收到传感器位姿数据

        // 传感器检测范围
        Eigen::Vector3d localSensorRange_ {5.0, 5.0, 5.0};  // 视觉传感器局部检测范围(米): x, y, z方向
        Eigen::Vector3d localLidarRange_ {10.0, 10.0, 5.0};  // 激光雷达局部检测范围(米): x, y, z方向

        // 激光雷达点云数据
        sensor_msgs::PointCloud2ConstPtr latestCloud_;  // 最新接收的点云消息(ROS格式)
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud_ = NULL;  // 激光雷达点云数据(PCL格式)
        std::vector<onboardDetector::Cluster> lidarClusters_;  // 激光雷达聚类结果

        // ========== 检测器数据 ==========
        // UV(图像平面)检测数据
        std::vector<onboardDetector::box3D> uvBBoxes_;  // UV检测器生成的3D边界框
        int projPointsNum_ = 0;  // 投影点数量
        std::vector<Eigen::Vector3d> projPoints_;  // 从深度图像投影得到的3D点云
        std::vector<double> pointsDepth_;  // 点云深度值列表

        // DBSCAN聚类检测数据
        std::vector<Eigen::Vector3d> filteredDepthPoints_;  // 过滤后的深度点云数据
        std::vector<onboardDetector::box3D> dbBBoxes_;  // DBSCAN聚类生成的3D边界框
        std::vector<std::vector<Eigen::Vector3d>> pcClustersVisual_;  // 视觉点云聚类(每个聚类是一个点集)
        std::vector<Eigen::Vector3d> pcClusterCentersVisual_;  // 视觉点云聚类中心
        std::vector<Eigen::Vector3d> pcClusterStdsVisual_;  // 视觉点云聚类在各轴的标准差

        // 融合过滤后的检测数据
        std::vector<onboardDetector::box3D> filteredBBoxesBeforeYolo_;  // YOLO融合前的过滤边界框
        std::vector<onboardDetector::box3D> filteredBBoxes_;  // 融合UV和DBSCAN后的过滤边界框
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClusters_;  // 融合过滤后的点云聚类
        std::vector<Eigen::Vector3d> filteredPcClusterCenters_;  // 过滤后点云聚类中心
        std::vector<Eigen::Vector3d> filteredPcClusterStds_;  // 过滤后点云聚类在各轴的标准差

        // 多传感器检测结果
        std::vector<onboardDetector::box3D> visualBBoxes_;  // 视觉相机检测的边界框
        std::vector<onboardDetector::box3D> lidarBBoxes_;  // 激光雷达检测的边界框(包含静态和动态)

        // 跟踪和分类结果
        std::vector<onboardDetector::box3D> trackedBBoxes_;  // 卡尔曼滤波跟踪后的边界框
        std::vector<onboardDetector::box3D> dynamicBBoxes_;  // 分类为动态的障碍物边界框

        // ========== 跟踪和数据关联数据 ==========
        bool newDetectFlag_;  // 新检测标志, 标识是否有新的检测结果

        // 历史数据队列(用于数据关联和跟踪)
        std::vector<std::deque<onboardDetector::box3D>> boxHist_;  // 边界框历史队列: 每个当前目标对应的历史边界框序列
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHist_;  // 点云聚类历史队列: 每个当前聚类对应的历史点云序列
        std::vector<std::deque<Eigen::Vector3d>> pcCenterHist_;  // 点云中心历史队列: 每个当前聚类对应的历史中心位置序列

        // 卡尔曼滤波器
        std::vector<onboardDetector::kalman_filter> filters_;  // 卡尔曼滤波器数组: 每个跟踪目标对应一个滤波器

        // ========== YOLO检测结果 ==========
        vision_msgs::Detection2DArray yoloDetectionResults_;  // YOLO检测的2D边界框结果
        cv::Mat detectedColorImage_;  // 带有检测框标注的彩色图像

    public:
        // ========== 构造函数和初始化 ==========
        /**
         * @brief 默认构造函数
         */
        dynamicDetector();

        /**
         * @brief 带节点句柄的构造函数
         * @param nh ROS节点句柄
         */
        dynamicDetector(const ros::NodeHandle& nh);

        /**
         * @brief 初始化检测器
         * @param nh ROS节点句柄
         * 功能: 完成参数加载、发布器注册、回调函数注册等初始化工作
         */
        void initDetector(const ros::NodeHandle& nh);

        /**
         * @brief 初始化参数
         * 功能: 从ROS参数服务器加载所有配置参数
         */
        void initParam();

        /**
         * @brief 注册发布器
         * 功能: 创建所有ROS话题发布器
         */
        void registerPub();

        /**
         * @brief 注册回调函数
         * 功能: 注册所有订阅器和定时器的回调函数
         */
        void registerCallback();

        // ========== ROS服务 ==========
        /**
         * @brief 获取动态障碍物服务回调函数
         * @param req 服务请求(空)
         * @param res 服务响应, 包含动态障碍物信息
         * @return 服务调用是否成功
         * 功能: 响应外部对动态障碍物信息的查询请求
         */
		bool getDynamicObstacles(onboard_detector_lv::GetDynamicObstacles::Request& req,
								 onboard_detector_lv::GetDynamicObstacles::Response& res);

        // ========== ROS回调函数 ==========
        /**
         * @brief 深度图像与位姿同步回调函数
         * @param img 深度图像消息
         * @param pose 位姿消息
         * 功能: 接收同步的深度图像和机器人位姿数据
         */
        void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);

        /**
         * @brief 深度图像与里程计同步回调函数
         * @param img 深度图像消息
         * @param odom 里程计消息
         * 功能: 接收同步的深度图像和机器人里程计数据
         */
        void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);

        /**
         * @brief 激光雷达点云与位姿同步回调函数
         * @param cloudMsg 点云消息
         * @param pose 位姿消息
         * 功能: 接收同步的激光雷达点云和机器人位姿数据
         */
        void lidarPoseCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const geometry_msgs::PoseStampedConstPtr& pose);

        /**
         * @brief 激光雷达点云与里程计同步回调函数
         * @param cloudMsg 点云消息
         * @param odom 里程计消息
         * 功能: 接收同步的激光雷达点云和机器人里程计数据
         */
        void lidarOdomCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const nav_msgs::OdometryConstPtr& odom);

        /**
         * @brief 彩色图像回调函数
         * @param img 彩色图像消息
         * 功能: 接收彩色图像数据, 用于YOLO检测和可视化
         */
        void colorImgCB(const sensor_msgs::ImageConstPtr& img);

        /**
         * @brief YOLO检测结果回调函数
         * @param detections YOLO检测的2D边界框数组
         * 功能: 接收YOLO神经网络的目标检测结果, 用于融合判断
         */
        void yoloDetectionCB(const vision_msgs::Detection2DArrayConstPtr& detections);

        /**
         * @brief 视觉检测定时器回调函数
         * @param event 定时器事件
         * 功能: 周期性执行视觉检测流程(UV检测和DBSCAN聚类)
         */
        void detectionCB(const ros::TimerEvent&);

        /**
         * @brief 激光雷达检测定时器回调函数
         * @param event 定时器事件
         * 功能: 周期性执行激光雷达检测流程
         */
        void lidarDetectionCB(const ros::TimerEvent&);

        /**
         * @brief 跟踪定时器回调函数
         * @param event 定时器事件
         * 功能: 周期性执行目标跟踪和数据关联
         */
        void trackingCB(const ros::TimerEvent&);

        /**
         * @brief 分类定时器回调函数
         * @param event 定时器事件
         * 功能: 周期性执行动静态分类
         */
        void classificationCB(const ros::TimerEvent&);

        /**
         * @brief 可视化定时器回调函数
         * @param event 定时器事件
         * 功能: 周期性发布可视化数据到RViz
         */
        void visCB(const ros::TimerEvent&);

        // ========== 检测功能函数 ==========
        /**
         * @brief UV检测
         * 功能: 在图像平面(UV空间)进行目标检测, 生成初始边界框
         */
        void uvDetect();

        /**
         * @brief DBSCAN聚类检测
         * 功能: 对投影的3D点云进行DBSCAN聚类, 生成聚类边界框
         */
        void dbscanDetect();

        /**
         * @brief 激光雷达检测
         * 功能: 处理激光雷达点云, 进行聚类检测
         */
        void lidarDetect();

        /**
         * @brief 过滤激光雷达和视觉边界框
         * 功能: 融合激光雷达和视觉检测结果, 通过IOU阈值过滤冗余检测
         */
        void filterLVBBoxes();

        /**
         * @brief 转换UV边界框坐标
         * @param bboxes 输入输出参数, 需要转换的边界框数组
         * 功能: 将边界框从相机坐标系转换到世界坐标系
         */
        void transformUVBBoxes(std::vector<onboardDetector::box3D>& bboxes);

        // ========== 视觉DBSCAN检测器辅助函数 ==========
        /**
         * @brief 投影深度图像
         * 功能: 将深度图像投影到3D空间, 生成点云
         */
        void projectDepthImage();

        /**
         * @brief 过滤点云
         * @param points 输入点云
         * @param filteredPoints 输出过滤后的点云
         * 功能: 根据高度和范围阈值过滤点云
         */
        void filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);

        /**
         * @brief 点云聚类并生成边界框
         * @param points 输入点云
         * @param bboxes 输出边界框数组
         * @param pcClusters 输出点云聚类(每个聚类是一个点集)
         * @param pcClusterCenters 输出聚类中心
         * @param pcClusterStds 输出聚类标准差
         * 功能: 使用DBSCAN对点云聚类, 并为每个聚类生成3D边界框
         */
        void clusterPointsAndBBoxes(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters, std::vector<Eigen::Vector3d>& pcClusterCenters, std::vector<Eigen::Vector3d>& pcClusterStds);

        /**
         * @brief 体素滤波
         * @param points 输入点云
         * @param filteredPoints 输出降采样后的点云
         * 功能: 使用体素网格进行点云降采样
         */
        void voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);

        // ========== 检测辅助函数 ==========
        /**
         * @brief 计算点云聚类特征
         * @param pcCluster 输入点云聚类
         * @param pcClusterCenter 输出聚类中心
         * @param pcClusterStd 输出聚类在各轴的标准差
         * 功能: 计算点云聚类的统计特征(中心和标准差)
         */
        void calcPcFeat(const std::vector<Eigen::Vector3d>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd);

        /**
         * @brief 计算两个边界框的IOU(交并比)
         * @param box1 第一个边界框
         * @param box2 第二个边界框
         * @param ignoreZmin 是否忽略Z轴最小值(默认false)
         * @return IOU值(0-1之间)
         * 功能: 计算3D边界框的体积交并比, 用于融合判断
         */
        double calBoxIOU(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2, bool ignoreZmin=false);

        // ========== 数据关联和跟踪函数 ==========
        /**
         * @brief 边界框数据关联
         * @param bestMatch 输出最佳匹配结果, bestMatch[i]表示当前帧第i个检测对应的历史目标索引(-1表示新目标)
         * 功能: 将当前帧检测结果与历史跟踪目标进行关联匹配
         */
        void boxAssociation(std::vector<int>& bestMatch);

        /**
         * @brief 边界框数据关联辅助函数
         * @param bestMatch 输出最佳匹配结果
         * 功能: 执行具体的数据关联算法
         */
        void boxAssociationHelper(std::vector<int>& bestMatch);

        /**
         * @brief 生成特征向量辅助函数
         * @param boxes 边界框数组
         * @param pcCenters 点云中心数组
         * @param feature 输出特征向量数组
         * 功能: 为每个边界框生成用于数据关联的特征向量(位置、尺寸等)
         */
        void genFeatHelper(const std::vector<onboardDetector::box3D>& boxes, const std::vector<Eigen::Vector3d>& pcCenters, std::vector<Eigen::VectorXd>& feature);

        /**
         * @brief 获取前一帧的边界框
         * @param prevBoxes 输出前一帧边界框
         * @param prevPcCenters 输出前一帧点云中心
         * 功能: 从历史记录中提取前一帧的检测结果
         */
        void getPrevBBoxes(std::vector<onboardDetector::box3D>& prevBoxes, std::vector<Eigen::Vector3d>& prevPcCenters);

        /**
         * @brief 线性运动预测
         * @param propedBoxes 输出预测的边界框
         * @param propedPcCenters 输出预测的点云中心
         * 功能: 基于匀速运动模型预测当前帧目标位置
         */
        void linearProp(std::vector<onboardDetector::box3D>& propedBoxes, std::vector<Eigen::Vector3d>& propedPcCenters);

        /**
         * @brief 查找最佳匹配
         * @param prevBBoxes 前一帧边界框
         * @param prevBoxesFeat 前一帧特征
         * @param propedBoxes 预测的边界框
         * @param propedBoxesFeat 预测的特征
         * @param currBoxesFeat 当前帧特征
         * @param bestMatch 输出最佳匹配结果
         * 功能: 基于特征距离进行数据关联, 找到当前检测与历史目标的最佳匹配
         */
        void findBestMatch(const std::vector<onboardDetector::box3D>& prevBBoxes, const std::vector<Eigen::VectorXd>& prevBoxesFeat, const std::vector<onboardDetector::box3D>& propedBoxes, const std::vector<Eigen::VectorXd>& propedBoxesFeat, const std::vector<Eigen::VectorXd>& currBoxesFeat, std::vector<int>& bestMatch);

        /**
         * @brief 卡尔曼滤波并更新历史
         * @param bestMatch 数据关联结果
         * 功能: 对匹配的目标执行卡尔曼滤波更新, 并更新历史记录队列
         */
        void kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch);

        /**
         * @brief 卡尔曼滤波矩阵初始化(速度模型)
         * @param currDetectedBBox 当前检测的边界框
         * @param states 输出状态向量
         * @param A 输出状态转移矩阵
         * @param B 输出控制输入矩阵
         * @param H 输出观测矩阵
         * @param P 输出状态协方差矩阵
         * @param Q 输出过程噪声协方差矩阵
         * @param R 输出观测噪声协方差矩阵
         * 功能: 初始化基于速度模型的卡尔曼滤波矩阵
         */
        void kalmanFilterMatrixVel(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R);

        /**
         * @brief 卡尔曼滤波矩阵初始化(加速度模型)
         * @param currDetectedBBox 当前检测的边界框
         * @param states 输出状态向量
         * @param A 输出状态转移矩阵
         * @param B 输出控制输入矩阵
         * @param H 输出观测矩阵
         * @param P 输出状态协方差矩阵
         * @param Q 输出过程噪声协方差矩阵
         * @param R 输出观测噪声协方差矩阵
         * 功能: 初始化基于加速度模型的卡尔曼滤波矩阵
         */
        void kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R);

        /**
         * @brief 获取卡尔曼观测向量(速度模型)
         * @param currDetectedBBox 当前检测的边界框
         * @param bestMatchIdx 最佳匹配的历史目标索引
         * @param Z 输出观测向量
         * 功能: 从当前检测和历史数据中构造卡尔曼滤波的观测向量
         */
        void getKalmanObservationVel(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z);

        /**
         * @brief 获取卡尔曼观测向量(加速度模型)
         * @param currDetectedBBox 当前检测的边界框
         * @param bestMatchIdx 最佳匹配的历史目标索引
         * @param Z 输出观测向量
         * 功能: 从当前检测和历史数据中构造卡尔曼滤波的观测向量(包含加速度)
         */
        void getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z);


        // ========== 可视化函数 ==========
        /**
         * @brief 获取动态点云
         * @param dynamicPc 输出动态障碍物对应的点云
         * 功能: 从检测结果中提取属于动态障碍物的点云数据
         */
        void getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc);

        /**
         * @brief 发布UV图像
         * 功能: 发布深度图和UV检测相关的可视化图像
         */
        void publishUVImages();

        /**
         * @brief 发布彩色图像
         * 功能: 发布带有检测框标注的彩色图像
         */
        void publishColorImages();

        /**
         * @brief 发布点云
         * @param points 点云数据
         * @param publisher 发布器
         * 功能: 将点云数据发布到指定的ROS话题
         */
        void publishPoints(const std::vector<Eigen::Vector3d>& points, const ros::Publisher& publisher);

        /**
         * @brief 发布3D边界框
         * @param bboxes 边界框数组
         * @param publisher 发布器
         * @param r 红色分量(0-1)
         * @param g 绿色分量(0-1)
         * @param b 蓝色分量(0-1)
         * 功能: 将3D边界框以MarkerArray形式发布到RViz
         */
        void publish3dBox(const std::vector<onboardDetector::box3D>& bboxes, const ros::Publisher& publisher, double r, double g, double b);

        /**
         * @brief 发布历史轨迹
         * 功能: 发布跟踪目标的历史运动轨迹
         */
        void publishHistoryTraj();

        /**
         * @brief 发布速度可视化
         * 功能: 发布动态障碍物的速度向量可视化
         */
        void publishVelVis();

        /**
         * @brief 发布激光雷达聚类
         * 功能: 发布激光雷达聚类结果
         */
        void publishLidarClusters();

        /**
         * @brief 发布过滤后的点云
         * 功能: 发布经过融合过滤后的点云
         */
        void publishFilteredPoints();

        /**
         * @brief 发布原始动态点云
         * 功能: 发布未经过滤的动态障碍物点云
         */
        void publishRawDynamicPoints();

        // ========== 工具函数 ==========
        /**
         * @brief 转换边界框坐标
         * @param center 边界框中心(原坐标系)
         * @param size 边界框尺寸
         * @param position 坐标系平移
         * @param orientation 坐标系旋转
         * @param newCenter 输出转换后的边界框中心
         * @param newSize 输出转换后的边界框尺寸
         * 功能: 将边界框从一个坐标系转换到另一个坐标系
         */
        void transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                                  Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize);

        /**
         * @brief 获取最佳重叠边界框
         * @param currBBox 当前边界框
         * @param targetBBoxes 目标边界框数组
         * @param bestIOU 输出最佳IOU值
         * @return 最佳匹配的边界框索引
         * 功能: 在目标边界框数组中找到与当前边界框IOU最大的那个
         */
        int getBestOverlapBBox(const onboardDetector::box3D& currBBox, const std::vector<onboardDetector::box3D>& targetBBoxes, double& bestIOU);

        // ========== 外部接口函数 ==========
        /**
         * @brief 获取动态障碍物
         * @param incomeDynamicBBoxes 输出动态障碍物边界框数组
         * @param robotSize 机器人尺寸(用于碰撞检查和膨胀), 默认为(0,0,0)
         * 功能: 供外部规划器调用, 获取当前检测到的所有动态障碍物
         */
        void getDynamicObstacles(std::vector<onboardDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));

        /**
         * @brief 获取动态障碍物历史轨迹
         * @param posHist 输出位置历史(每个障碍物对应一个位置序列)
         * @param velHist 输出速度历史(每个障碍物对应一个速度序列)
         * @param sizeHist 输出尺寸历史(每个障碍物对应一个尺寸序列)
         * @param robotSize 机器人尺寸(用于碰撞检查和膨胀), 默认为(0,0,0)
         * 功能: 供外部规划器调用, 获取动态障碍物的历史运动轨迹, 用于轨迹预测
         */
        void getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist,
									 std::vector<std::vector<Eigen::Vector3d>>& velHist,
									 std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));

        // ========== 内联辅助函数 ==========
        /**
         * @brief 判断位置是否在过滤范围内
         * @param pos 待判断的3D位置
         * @return 是否在范围内
         * 功能: 判断点是否在传感器检测范围内(相对于机器人当前位置)
         */
        bool isInFilterRange(const Eigen::Vector3d& pos);

        /**
         * @brief 位置转换为体素索引
         * @param pos 3D位置
         * @param idx 输出体素索引
         * @param res 体素分辨率
         * 功能: 将世界坐标转换为体素网格索引
         */
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res);

        /**
         * @brief 体素索引转换为线性地址
         * @param idx 体素索引
         * @param res 体素分辨率
         * @return 线性地址
         * 功能: 将3D体素索引转换为1D线性地址(用于哈希表存储)
         */
        int indexToAddress(const Eigen::Vector3i& idx, double res);

        /**
         * @brief 位置转换为线性地址
         * @param pos 3D位置
         * @param res 体素分辨率
         * @return 线性地址
         * 功能: 将世界坐标直接转换为线性地址
         */
        int posToAddress(const Eigen::Vector3d& pos, double res);

        /**
         * @brief 体素索引转换为位置
         * @param idx 体素索引
         * @param pos 输出3D位置
         * @param res 体素分辨率
         * 功能: 将体素网格索引转换为世界坐标(体素中心)
         */
        void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res);

        /**
         * @brief 从位姿消息获取相机位姿(重载1)
         * @param pose 位姿消息
         * @param camPoseDepthMatrix 输出深度相机位姿矩阵
         * @param camPoseColorMatrix 输出彩色相机位姿矩阵
         * 功能: 将ROS位姿消息转换为相机坐标系的齐次变换矩阵
         */
        void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix);

        /**
         * @brief 从里程计消息获取相机位姿(重载2)
         * @param odom 里程计消息
         * @param camPoseDepthMatrix 输出深度相机位姿矩阵
         * @param camPoseColorMatrix 输出彩色相机位姿矩阵
         * 功能: 将ROS里程计消息转换为相机坐标系的齐次变换矩阵
         */
        void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix);

        /**
         * @brief 从位姿消息获取激光雷达位姿(重载1)
         * @param pose 位姿消息
         * @param lidarPoseMatrix 输出激光雷达位姿矩阵
         * 功能: 将ROS位姿消息转换为激光雷达坐标系的齐次变换矩阵
         */
        void getLidarPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& lidarPoseMatrix);

        /**
         * @brief 从里程计消息获取激光雷达位姿(重载2)
         * @param odom 里程计消息
         * @param lidarPoseMatrix 输出激光雷达位姿矩阵
         * 功能: 将ROS里程计消息转换为激光雷达坐标系的齐次变换矩阵
         */
        void getLidarPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& lidarPoseMatrix);

        /**
         * @brief Eigen点转换为DBSCAN点
         * @param p Eigen格式的3D点
         * @return DBSCAN格式的点
         * 功能: 数据格式转换, 用于DBSCAN聚类算法
         */
        onboardDetector::Point eigenToDBPoint(const Eigen::Vector3d& p);

        /**
         * @brief DBSCAN点转换为Eigen点
         * @param pDB DBSCAN格式的点
         * @return Eigen格式的3D点
         * 功能: 数据格式转换, 从DBSCAN结果提取点坐标
         */
        Eigen::Vector3d dbPointToEigen(const onboardDetector::Point& pDB);

        /**
         * @brief Eigen点向量转换为DBSCAN点向量
         * @param points Eigen格式的点云
         * @param pointsDB 输出DBSCAN格式的点云
         * @param size 点的数量
         * 功能: 批量转换点云数据格式
         */
        void eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::Point>& pointsDB, int size);       
    };


    // ========== 内联函数实现 ==========
    /**
     * @brief 判断点是否在传感器有效检测范围内
     * 检测范围为以机器人当前位置为中心的矩形区域
     */
    inline bool dynamicDetector::isInFilterRange(const Eigen::Vector3d& pos){
        if ((pos(0) >= this->position_(0) - this->localSensorRange_(0)) and (pos(0) <= this->position_(0) + this->localSensorRange_(0)) and 
            (pos(1) >= this->position_(1) - this->localSensorRange_(1)) and (pos(1) <= this->position_(1) + this->localSensorRange_(1)) and 
            (pos(2) >= this->position_(2) - this->localSensorRange_(2)) and (pos(2) <= this->position_(2) + this->localSensorRange_(2))){
            return true;
        }
        else{
            return false;
        }        
    }

    /**
     * @brief 世界坐标转体素索引
     * 将连续的世界坐标离散化为体素网格索引
     */
    inline void dynamicDetector::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res){
        idx(0) = floor( (pos(0) - this->position_(0) + localSensorRange_(0)) / res);
        idx(1) = floor( (pos(1) - this->position_(1) + localSensorRange_(1)) / res);
        idx(2) = floor( (pos(2) - this->position_(2) + localSensorRange_(2)) / res);
    }

    /**
     * @brief 3D体素索引转1D线性地址
     * 使用行优先顺序: address = x * (Ny * Nz) + y * Nz + z
     */
    inline int dynamicDetector::indexToAddress(const Eigen::Vector3i& idx, double res){
        return idx(0) * ceil(2*this->localSensorRange_(1)/res) * ceil(2*this->localSensorRange_(2)/res) + idx(1) * ceil(2*this->localSensorRange_(2)/res) + idx(2);
        // return idx(0) * ceil(this->localSensorRange_(0)/res) + idx(1) * ceil(this->localSensorRange_(1)/res) + idx(2);
    }

    /**
     * @brief 世界坐标直接转线性地址
     * 组合了posToIndex和indexToAddress两个操作
     */
    inline int dynamicDetector::posToAddress(const Eigen::Vector3d& pos, double res){
        Eigen::Vector3i idx;
        this->posToIndex(pos, idx, res);
        return this->indexToAddress(idx, res);
    }

    /**
     * @brief 体素索引转世界坐标
     * 返回体素中心位置 (idx + 0.5) * res
     */
    inline void dynamicDetector::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res){
		pos(0) = (idx(0) + 0.5) * res - localSensorRange_(0) + this->position_(0);
		pos(1) = (idx(1) + 0.5) * res - localSensorRange_(1) + this->position_(1);
		pos(2) = (idx(2) + 0.5) * res - localSensorRange_(2) + this->position_(2);
	}
    
    /**
     * @brief 从位姿消息计算相机位姿矩阵
     * 计算流程: 世界坐标系 -> 机体坐标系 -> 相机坐标系
     */
    inline void dynamicDetector::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix){
        // 提取四元数并转换为旋转矩阵
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // 构建世界坐标系到机体坐标系的变换矩阵
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;  // 旋转部分
        map2body(0, 3) = pose->pose.position.x;  // 平移部分
        map2body(1, 3) = pose->pose.position.y;
        map2body(2, 3) = pose->pose.position.z;
        map2body(3, 3) = 1.0;

        // 计算相机位姿: T_map_to_cam = T_map_to_body * T_body_to_cam
        camPoseDepthMatrix = map2body * this->body2CamDepth_;
        camPoseColorMatrix = map2body * this->body2CamColor_;
    }

    /**
     * @brief 从里程计消息计算相机位姿矩阵
     * 与上一个函数功能相同, 但输入为里程计消息
     */
    inline void dynamicDetector::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix){
        // 提取四元数并转换为旋转矩阵
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // 构建世界坐标系到机体坐标系的变换矩阵
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;  // 旋转部分
        map2body(0, 3) = odom->pose.pose.position.x;  // 平移部分
        map2body(1, 3) = odom->pose.pose.position.y;
        map2body(2, 3) = odom->pose.pose.position.z;
        map2body(3, 3) = 1.0;

        // 计算相机位姿: T_map_to_cam = T_map_to_body * T_body_to_cam
        camPoseDepthMatrix = map2body * this->body2CamDepth_;
        camPoseColorMatrix = map2body * this->body2CamColor_;
    }

    /**
     * @brief 从位姿消息计算激光雷达位姿矩阵
     * 计算流程: 世界坐标系 -> 机体坐标系 -> 激光雷达坐标系
     */
    inline void dynamicDetector::getLidarPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& lidarPoseMatrix){
        // 提取四元数并转换为旋转矩阵
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // 构建世界坐标系到机体坐标系的变换矩阵
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;  // 旋转部分
        map2body(0, 3) = pose->pose.position.x;  // 平移部分
        map2body(1, 3) = pose->pose.position.y;
        map2body(2, 3) = pose->pose.position.z;
        map2body(3, 3) = 1.0;

        // 计算激光雷达位姿: T_map_to_lidar = T_map_to_body * T_body_to_lidar
        lidarPoseMatrix = map2body * this->body2Lidar_;
    }

    /**
     * @brief 从里程计消息计算激光雷达位姿矩阵
     * 与上一个函数功能相同, 但输入为里程计消息
     */
    inline void dynamicDetector::getLidarPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& lidarPoseMatrix){
        // 提取四元数并转换为旋转矩阵
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // 构建世界坐标系到机体坐标系的变换矩阵
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;  // 旋转部分
        map2body(0, 3) = odom->pose.pose.position.x;  // 平移部分
        map2body(1, 3) = odom->pose.pose.position.y;
        map2body(2, 3) = odom->pose.pose.position.z;
        map2body(3, 3) = 1.0;

        // 计算激光雷达位姿: T_map_to_lidar = T_map_to_body * T_body_to_lidar
        lidarPoseMatrix = map2body * this->body2Lidar_;
    }
    
    /**
     * @brief Eigen格式点转DBSCAN格式点
     * 初始化聚类ID为-1(未分类)
     */
    inline onboardDetector::Point dynamicDetector::eigenToDBPoint(const Eigen::Vector3d& p){
        onboardDetector::Point pDB;
        pDB.x = p(0);
        pDB.y = p(1);
        pDB.z = p(2);
        pDB.clusterID = -1;
        return pDB;
    }

    /**
     * @brief DBSCAN格式点转Eigen格式点
     * 提取xyz坐标, 忽略聚类ID
     */
    inline Eigen::Vector3d dynamicDetector::dbPointToEigen(const onboardDetector::Point& pDB){
        Eigen::Vector3d p;
        p(0) = pDB.x;
        p(1) = pDB.y;
        p(2) = pDB.z;
        return p;
    }

    /**
     * @brief 批量转换点云格式(Eigen -> DBSCAN)
     * 遍历点云数组, 逐个转换格式
     */
    inline void dynamicDetector::eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::Point>& pointsDB, int size){
        for (int i=0; i<size; ++i){
            Eigen::Vector3d p = points[i];
            onboardDetector::Point pDB = this->eigenToDBPoint(p);
            pointsDB.push_back(pDB);
        }
    }
}

#endif
