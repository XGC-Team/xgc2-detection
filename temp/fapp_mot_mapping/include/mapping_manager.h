/**
 * @file mapping_manager.h
 * @brief 多目标跟踪与建图管理器头文件
 * @details 该文件定义了基于点云的动态目标检测、跟踪和建图的核心类和数据结构
 *          使用DBSCAN聚类、EKF滤波和匈牙利算法进行多目标跟踪
 */

#ifndef _mot_mapping_H_
#define _mot_mapping_H_

#include <chrono>
#include <queue>
// ROS 相关头文件
#include <message_filters/subscriber.h>                           // 消息过滤订阅器
#include <message_filters/sync_policies/approximate_time.h>       // 近似时间同步策略
#include <message_filters/sync_policies/exact_time.h>             // 精确时间同步策略
#include <message_filters/time_synchronizer.h>                    // 时间同步器
#include <sensor_msgs/PointCloud2.h>                              // 点云消息类型
#include <geometry_msgs/Point.h>                                  // 几何点消息类型
#include <nav_msgs/Odometry.h>                                    // 里程计消息类型
#include <visualization_msgs/Marker.h>                            // 可视化标记消息
#include <visualization_msgs/MarkerArray.h>                       // 可视化标记数组消息
#include <fapp_obj_state_msgs/ObjectsStates.h>                    // 自定义目标状态消息
#include <fapp_obj_state_msgs/State.h>                            // 自定义状态消息
// PCL 点云库相关头文件
#include <pcl/point_types.h>                                      // 点类型定义
#include <pcl/common/common.h>                                    // 通用函数
#include <pcl/io/pcd_io.h>                                        // PCD文件读写
#include <pcl/filters/voxel_grid.h>                               // 体素网格滤波器
#include <pcl/features/normal_3d.h>                               // 3D法线特征
#include <pcl_conversions/pcl_conversions.h>                      // PCL与ROS消息转换
#include <pcl/range_image/range_image.h>                          // 距离图像
#include <pcl/search/search.h>                                    // 搜索接口
#include <pcl/search/kdtree.h>                                    // KD树搜索
#include <pcl/segmentation/extract_clusters.h>                    // 聚类提取
#include <pcl/surface/convex_hull.h>                              // 凸包计算
#include <pcl/filters/extract_indices.h>                          // 索引提取滤波器
#include <pcl/ModelCoefficients.h>                                // 模型系数
#include <pcl/segmentation/sac_segmentation.h>                    // RANSAC分割

// Eigen 线性代数库
#include <eigen3/Eigen/Core>                                      // 核心功能
#include <eigen3/Eigen/Dense>                                     // 稠密矩阵
#include <eigen3/Eigen/Geometry>                                  // 几何变换
#include <eigen3/unsupported/Eigen/MatrixFunctions>               // 矩阵函数
// 算法相关头文件
#include "DBSCAN_kdtree.h"                                        // 基于KD树的DBSCAN聚类算法
#include "Hungarian.h"                                            // 匈牙利算法（用于数据关联）
#include "target_ekf.hpp"                                         // 目标扩展卡尔曼滤波器
#include "ikd_Tree.h"                                             // 增量式KD树
#include "ikd_Tree_impl.h"                                        // 增量式KD树实现

using namespace std;

// 点云数据类型定义
typedef pcl::PointXYZ PointType;                                                    // 基本点类型（包含X,Y,Z坐标）
typedef pcl::PointCloud<PointType> Points;                                          // 点云类型
typedef pcl::PointCloud<PointType>::Ptr PointsPtr;                                  // 点云智能指针类型
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;        // 使用Eigen内存对齐的点向量

/**
 * @struct MapParam
 * @brief 地图参数结构体
 * @details 存储DBSCAN聚类参数、动态检测参数、雷达参数和ROS话题参数
 */
struct MapParam {
  // DBSCAN聚类参数
  int core_pts;                    // DBSCAN核心点所需的最小邻居数量
  double tolerance;                // DBSCAN聚类的距离容差（米）
  int min_cluster;                 // 聚类的最小点数阈值
  int max_cluster;                 // 聚类的最大点数阈值
  // 动态目标检测参数
  double thresh_dist;              // 动态检测的距离阈值（米）
  double thresh_var;               // 动态检测的方差阈值
  // 雷达参数
  Eigen::Vector3d Range;           // 雷达的有效探测范围（X,Y,Z方向）
  // ROS话题参数
  std::string odom_topic;          // 里程计话题名称
  std::string lidar_topic;         // 雷达点云话题名称
  std::string cloud_topic;         // 订阅点云话题名称
};

/**
 * @struct MapData
 * @brief 地图数据结构体
 * @details 存储当前机器人的位姿信息（位置和姿态）
 */
struct MapData {
  Eigen::Matrix3d R;               // 当前IMU的旋转矩阵（姿态）
  Eigen::Vector3d T;               // 当前IMU的位置向量
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏，确保在动态分配时正确对齐
};

/**
 * @struct ObjectState
 * @brief 目标状态结构体
 * @details 存储检测到的动态目标的完整状态信息
 */
struct ObjectState {
  int id;                          // 目标的唯一标识ID
  Eigen::Vector3d position;        // 目标的3D位置（世界坐标系）
  Eigen::Vector3d velocity;        // 目标的3D速度向量
  Eigen::Vector3d size;            // 目标的包围盒尺寸（长、宽、高）
};

namespace mot_mapping {

/**
 * @class MappingRos
 * @brief 多目标跟踪与建图的ROS接口类
 * @details 该类实现了基于点云的动态目标检测、跟踪和建图功能，主要包括：
 *          1. 点云预处理和体素滤波
 *          2. DBSCAN聚类进行目标检测
 *          3. EKF滤波器进行目标状态估计和跟踪
 *          4. 匈牙利算法进行数据关联
 *          5. iKD-Tree进行静态地图管理
 */
class MappingRos {

private:
  // ==================== 点云处理相关成员 ====================
  pcl::VoxelGrid<PointType> vox;                       // 体素网格滤波器，用于点云降采样
  DBSCANKdtreeCluster<PointType> dbscan;               // 基于KD树的DBSCAN聚类器，用于目标检测
  KD_TREE<PointType>::Ptr ikdtree_ptr;                 // 增量式KD树指针，用于高效的静态地图管理

  // ==================== 点云数据缓存 ====================
  PointsPtr Remaining_Points;                          // 剩余的静态点云（去除动态目标后）
  int frame_num;                                       // 当前帧编号
  std::deque<PointsPtr> previous_points;               // 历史点云队列，用于动态检测
  std::deque<PointsPtr> buffer;                        // 点云缓冲队列
  std::deque<PointsPtr> input_point;                   // 输入点云队列
  std::vector<Eigen::Vector3d> previous_p;             // 历史目标位置向量
  std::vector<Eigen::Vector3d> previous_v;             // 历史目标速度向量
  std::vector<pcl::PointIndices> deleted_indices;      // 被删除点的索引集合
  std::vector<BoxPointType> delete_boxes;              // 待删除的包围盒集合

  // ==================== 多目标跟踪相关成员 ====================
  std::vector<std::shared_ptr<Ekf>> trackers;          // EKF跟踪器向量，每个跟踪器对应一个目标
  std::vector<ObjectState> detections;                 // 当前帧检测到的目标状态集合

  // ==================== 时间戳 ====================
  ros::Time t_start;                                   // 系统启动时间戳

private:
  // ==================== ROS通信相关成员 ====================
  ros::NodeHandle &nh;                                 // ROS节点句柄引用
  ros::Timer ekf_predict_timer_;                       // EKF预测定时器，定期执行状态预测
  ros::Timer map_pub_timer_;                           // 地图发布定时器，定期发布地图数据
  ros::Publisher cloudPub;                             // 点云发布器
  ros::Publisher mapPub;                               // 地图发布器（包含动态目标）
  ros::Publisher staticMapPub;                         // 静态地图发布器
  ros::Publisher edgePub;                              // 边缘点发布器
  ros::Publisher objectPosePub;                        // 目标位姿发布器
  ros::Publisher statesPub;                            // 目标状态发布器
  int id;                                              // 当前跟踪器ID计数器

  // ==================== 消息同步相关类型定义 ====================
  // 定义点云和里程计的近似时间同步策略
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  // 定义同步器的智能指针类型
  typedef boost::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;  // 点云消息订阅器
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;         // 里程计消息订阅器

  SynchronizerCloudOdom sync_cloud_odom_;              // 点云与里程计的时间同步器

  // ==================== 数据管理 ====================
  unique_ptr<MapData> md_;                             // 地图数据智能指针（存储当前位姿）
  unique_ptr<MapParam> mp_;                            // 地图参数智能指针（存储配置参数）

  // ==================== 私有工具函数 ====================
  /**
   * @brief 计算两个点之间的欧式距离
   * @param p1 第一个点
   * @param p2 第二个点
   * @return 两点之间的欧式距离
   */
  float calc_dist(PointType p1, PointType p2);

  /**
   * @brief 生成包围盒
   * @param boxpoint 输出的包围盒对象
   * @param center_pt 包围盒中心点
   * @param box_lengths 包围盒的三个维度长度（长、宽、高）
   */
  void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths);

  /**
   * @brief 将点云从机体坐标系转换到世界坐标系
   * @param p_b 机体坐标系下的点云
   * @param p_w 世界坐标系下的点云（输出）
   */
  void pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w);

  /**
   * @brief 将点云从世界坐标系转换到机体坐标系
   * @param p_w 世界坐标系下的点云
   * @param p_b 机体坐标系下的点云（输出）
   * @param T 平移向量
   */
  void pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T);

  // ==================== 回调函数 ====================
  /**
   * @brief 点云和里程计同步回调函数
   * @details 处理同步的点云和里程计数据，执行动态目标检测和跟踪
   * @param msg 点云消息
   * @param odom 里程计消息
   */
  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& odom);

  /**
   * @brief EKF预测定时器回调函数
   * @details 定期执行所有跟踪器的状态预测步骤
   * @param e 定时器事件
   */
  void ekfPredictCallback(const ros::TimerEvent& e);

  /**
   * @brief 地图发布定时器回调函数
   * @details 定期发布静态地图和动态目标信息
   * @param e 定时器事件
   */
  void mapPubCallback(const ros::TimerEvent& e);

  /**
   * @brief 可视化函数
   * @details 可视化目标跟踪和数据关联的结果
   * @param pairs 检测目标与跟踪器的匹配对
   */
  void visualizeFunction(const std::vector<std::pair<int, int>> pairs);

  // ==================== 数据关联辅助函数 ====================
  /**
   * @brief 计算两个目标状态的交并比（Intersection over Union）
   * @details 用于数据关联中评估目标之间的相似度
   * @param state1 第一个目标状态
   * @param state2 第二个目标状态
   * @return IoU值，范围[0,1]，值越大表示重叠度越高
   */
  double iou(ObjectState state1, ObjectState state2);

public:
  // ==================== 公共接口函数 ====================
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄引用
   */
  MappingRos(ros::NodeHandle &nh);

  /**
   * @brief 析构函数
   * @details 负责释放资源和清理动态分配的内存
   */
  ~MappingRos();

  /**
   * @brief 初始化函数
   * @details 初始化所有ROS通信接口、参数加载、算法模块等
   *          包括：
   *          1. 从参数服务器加载配置参数
   *          2. 初始化发布器和订阅器
   *          3. 设置消息同步器
   *          4. 初始化DBSCAN聚类器和iKD-Tree
   *          5. 启动定时器
   */
  void init();

};

} // namespace mot_mapping

#endif // _mot_mapping_H_
