// 动态障碍物映射管理器实现文件
// 功能：基于点云的动态障碍物检测、聚类、跟踪和状态估计
#include "mapping_manager.h"

namespace mot_mapping {

// 构造函数：初始化ROS节点句柄
MappingRos::MappingRos(ros::NodeHandle &nh):nh(nh) {}

// 析构函数：清理跟踪器资源
MappingRos::~MappingRos() {std::cout << "Exit Tracker" << std::endl;}

/**
 * @brief 初始化映射系统
 * 功能：设置参数、初始化数据结构、创建发布者和订阅者、启动定时器
 */
void MappingRos::init() {
  // 初始化时间戳和计数器
  t_start = ros::Time::now();  // 记录系统启动时间
  frame_num = 1;  // 初始化帧计数器
  id = 0;  // 初始化物体ID计数器

  // 初始化点云容器和ikd-tree数据结构
  Remaining_Points.reset(new Points);  // 用于存储剩余点云
  ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));  // 参数：删除准则、平衡准则、下采样大小

  // 创建地图参数和数据对象
  mp_ = std::make_unique<MapParam>();  // 地图参数管理器
  md_ = std::make_unique<MapData>();   // 地图数据容器

  // 从参数服务器加载DBSCAN聚类参数
  nh.param("dbscan/core_pts", mp_->core_pts, 4);  // 核心点最小邻居数
  nh.param("dbscan/tolerance", mp_->tolerance, 0.02);  // 聚类容差距离(m)
  nh.param("dbscan/min_cluster", mp_->min_cluster, 20);  // 最小聚类点数
  nh.param("dbscan/max_cluster", mp_->max_cluster, 800);  // 最大聚类点数

  // 加载动态检测阈值参数
  nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);  // 平均距离阈值(m)
  nh.param("detection/thresh_var", mp_->thresh_var, 0.28);  // 距离方差阈值

  // 加载激光雷达感知范围参数
  nh.param("lidar/range_x", mp_->Range[0], 15.0);  // X方向范围(m)
  nh.param("lidar/range_y", mp_->Range[1], 15.0);  // Y方向范围(m)
  nh.param("lidar/range_z", mp_->Range[2], 1.0);   // Z方向范围(m)

  // 加载话题名称参数
  nh.param("ros/odom_topic", mp_->odom_topic, string("/odom"));  // 里程计话题
  nh.param("ros/lidar_topic", mp_->lidar_topic, string("/livox/lidar"));  // 点云话题
  nh.param("cloud_topic", mp_->cloud_topic, string("/fapp_map_generator/obj_cloud"));
  nh.param("odom_topic", mp_->odom_topic, string("/drone_0_visual_slam/odom"));

  // 创建ROS发布者
  cloudPub = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_points", 10);  // 动态点云发布者
  mapPub = nh.advertise<sensor_msgs::PointCloud2>("/map_ros", 10);  // 地图点云发布者
  staticMapPub = nh.advertise<sensor_msgs::PointCloud2>("/static_map", 10);  // 静态地图发布者
  edgePub = nh.advertise<visualization_msgs::MarkerArray>("/box_edge", 1000);  // 边界框发布者
  objectPosePub = nh.advertise<visualization_msgs::MarkerArray>("/object_pose", 10);  // 物体位姿发布者
  statesPub = nh.advertise<fapp_obj_state_msgs::ObjectsStates>("/states", 10);  // 物体状态发布者

  std::cout << "INIT!" << std::endl;

  // 创建点云和里程计的消息过滤器订阅者
  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, mp_->cloud_topic, 50));  // 点云订阅者，队列50
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, mp_->odom_topic, 25));  // 里程计订阅者，队列25

  // 创建点云和里程计的时间同步器
  sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
      MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));  // 同步队列大小100
  sync_cloud_odom_->registerCallback(boost::bind(&MappingRos::cloudOdomCallback, this, _1, _2));  // 注册同步回调函数

  // 创建定时器
  ekf_predict_timer_ = nh.createTimer(ros::Duration(0.02), &MappingRos::ekfPredictCallback, this);  // EKF预测定时器，50Hz
  map_pub_timer_ = nh.createTimer(ros::Duration(0.05), &MappingRos::mapPubCallback, this);  // 地图发布定时器，20Hz
}


/**
 * @brief 计算两点之间的欧氏距离的平方
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 距离的平方值
 */
float MappingRos::calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

/**
 * @brief 根据中心点和长度生成边界框
 * @param boxpoint 输出的边界框结构
 * @param center_pt 边界框中心点
 * @param box_lengths 边界框三个方向的半长度 [x_half, y_half, z_half]
 */
void MappingRos::generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];  // X方向半长度
    float &y_dist = box_lengths[1];  // Y方向半长度
    float &z_dist = box_lengths[2];  // Z方向半长度

    // 计算边界框的最小和最大顶点坐标
    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

/**
 * @brief 将点云从机体坐标系转换到世界坐标系
 * @param p_b 输入的机体坐标系点云
 * @param p_w 输出的世界坐标系点云
 * 注意：此函数实际上只进行了距离滤波，未进行真正的坐标变换
 */
void MappingRos::pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w) {
  int num = p_b->points.size();  // 获取点云数量
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    // 将点转换为Eigen向量
    p << p_b->points[i].x,
         p_b->points[i].y,
         p_b->points[i].z;
    // p = md_->R * p + md_->T;  // 原本的坐标变换（已注释）

    // 只保留距离机器人XY平面距离小于5米的点
    if ((p - md_->T).head(2).norm() < 5.0) {
        PointType pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];

        p_w->points.push_back(pt);
    }
  }
}

/**
 * @brief 将点云从世界坐标系转换到机体坐标系
 * @param p_w 输入的世界坐标系点云
 * @param p_b 输出的机体坐标系点云
 * @param T 平移向量（参数未使用）
 */
void MappingRos::pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T) {
  int num = p_w->points.size();  // 获取点云数量
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    // 将点转换为Eigen向量
    p << p_w->points[i].x,
         p_w->points[i].y,
         p_w->points[i].z;
    // 减去当前位置T，得到相对坐标
    p = (p - md_->T);

    PointType pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p_b->points.push_back(pt);
  }
}


/**
 * @brief 点云和里程计同步回调函数
 * @param msg 点云消息
 * @param odom 里程计消息
 * 功能：处理同步的点云和位姿数据，进行滤波、聚类、检测和跟踪
 */
void MappingRos::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                                   const nav_msgs::OdometryConstPtr& odom) {
  // 将ROS点云消息转换为PCL格式
  PointsPtr latest_cloud(new Points);
  pcl::fromROSMsg(*msg, *latest_cloud);

  // 从里程计获取当前位置
  md_->T << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z;

  // 从里程计获取当前姿态（四元数转旋转矩阵）
  md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();

  // 初始化计时器，用于性能分析
  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  double compTime;
  // 清屏，准备输出调试信息
  printf("\033[2J");
  printf("\033[1;1H");

  // ========== 点云滤波 ==========
  PointsPtr cloud_filtered(new Points);
  vox.setInputCloud(latest_cloud);
  vox.setLeafSize(0.1, 0.1, 0.1);  // 设置体素滤波器分辨率为0.1m
  vox.filter(*cloud_filtered);  // 执行体素下采样滤波

  PointsPtr cloud_world(new Points);
  PointsPtr nonfilter_pts(new Points);
  PointsPtr PointToAdd(new Points);
  PointsPtr ClusterPoints(new Points);
  pointsBodyToWorld(cloud_filtered, cloud_world);
  pointsBodyToWorld(latest_cloud, nonfilter_pts);

  // ========== 点云缓冲区管理 ==========
  // 将非滤波点云添加到缓冲区，用于时间累积
  buffer.push_back(nonfilter_pts);
  if (buffer.size() > 1) {
    buffer.pop_front();  // 保持缓冲区大小为1，实现滑动窗口
  }
  // 合并缓冲区中的所有点云
  for (int i = 0; i < buffer.size(); ++i) {
    *ClusterPoints += *buffer[i];
  }
  // 对合并后的点云进行体素滤波（0.05m分辨率）
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*ClusterPoints);

  Remaining_Points = ClusterPoints;  // 保存剩余点云供后续处理

  PointsPtr All_Points(new Points);

  // 计算并输出滤波阶段耗时
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Filter Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  std::cout << "Input Size:" << ClusterPoints->size() << std::endl;

  // ========== 发布地图点云 ==========
  sensor_msgs::PointCloud2 map_ros;
  PointsPtr OutputPoints(new Points);
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*OutputPoints);

  // 转换为ROS消息并发布
  pcl::toROSMsg(*OutputPoints, map_ros);
  map_ros.header.frame_id = "world";
  mapPub.publish(map_ros);

  // ========== DBSCAN聚类 ==========
  // 使用DBSCAN算法对点云进行聚类，识别独立的物体
  std::vector<pcl::PointIndices> cluster_indices;
  dbscan.setCorePointMinPts(mp_->core_pts);  // 设置核心点最小邻居数
  dbscan.setClusterTolerance(mp_->tolerance);  // 设置聚类容差距离
  dbscan.setMinClusterSize(mp_->min_cluster);  // 设置最小聚类点数
  dbscan.setMaxClusterSize(mp_->max_cluster);  // 设置最大聚类点数
  dbscan.setInputCloud(ClusterPoints);
  dbscan.setSearchMethod();
  dbscan.extractNano(cluster_indices);  // 执行聚类提取

  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;
  // 计算并输出聚类阶段耗时
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Cluster Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  // ========== 初始化阶段 ==========
  // 前两帧用于初始化，不进行检测
  if (frame_num < 2){
    previous_points.push_back(cloud_world);
    frame_num++;
    return;
  }

  int pt_size = previous_points[0]->points.size();

  // ========== ikd-tree增量式地图维护 ==========
  // ikd-tree是一种增量式KD树，用于高效维护历史点云地图
  if (ikdtree_ptr->Root_Node == nullptr) {
    // 首次构建ikd-tree
    ikdtree_ptr->Build(previous_points[0]->points);
    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  else {
    // 增量式更新ikd-tree
    PointVector PointNoNeedDownsample;  // 不需要下采样的点
    bool need_add = true;

    // 遍历前一帧的所有点
    for (int i = 0; i < pt_size; ++i) {
      PointType mid_point;
      // 计算点所在体素的中心点（0.1m分辨率）
      mid_point.x = floor(previous_points[0]->points[i].x/0.1)*0.1 + 0.5 * 0.1;
      mid_point.y = floor(previous_points[0]->points[i].y/0.1)*0.1 + 0.5 * 0.1;
      mid_point.z = floor(previous_points[0]->points[i].z/0.1)*0.1 + 0.5 * 0.1;

      // 在ikd-tree中搜索最近邻点
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);

      // 如果最近邻点与体素中心距离超过半个体素，说明该点需要添加
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.1 ||
          fabs(points_near[0].y - mid_point.y) > 0.5 * 0.1 ||
          fabs(points_near[0].z - mid_point.z) > 0.5 * 0.1) {
        PointNoNeedDownsample.push_back(previous_points[0]->points[i]);
        PointToAdd->points.push_back(previous_points[0]->points[i]);
      }
    }

    // 将新点添加到ikd-tree
    ikdtree_ptr->Add_Points(PointNoNeedDownsample, false);
    input_point.push_back(PointToAdd);

    // 维护滑动窗口，保持最近25帧的点云
    if (input_point.size() > 25) {
      PointVector PointDelete;
      for (auto& delet_pt: input_point[0]->points) {
        PointDelete.push_back(delet_pt);
      }
      ikdtree_ptr->Delete_Points(PointDelete);  // 删除旧点
      input_point.pop_front();
    }

    // 更新前一帧点云
    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  // 计算并输出ikd-tree维护耗时
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "ikd-Tree Time Cost (ms)： " << compTime << std::endl;
  tic = std::chrono::high_resolution_clock::now();
  std::cout << "ikd-Tree Size: " << ikdtree_ptr->size() << std::endl;

  // 以下代码已注释：用于展平ikd-tree并发布历史点云
  // ikdtree_ptr->flatten(ikdtree_ptr->Root_Node, ikdtree_ptr->PCL_Storage, NOT_RECORD);
  // All_Points->points = ikdtree_ptr->PCL_Storage;
  // All_Points->width = All_Points->points.size();
  // All_Points->height = 1;
  // All_Points->is_dense = true;
  //
  // sensor_msgs::PointCloud2 previous_cloud;
  // pcl::toROSMsg(*All_Points, previous_cloud);
  // previous_cloud.header.frame_id = "world";
  // cloudPub.publish(previous_cloud);

  PointsPtr dynamic_points(new Points);  // 存储检测到的动态点云

  // ========== 动态障碍物检测 ==========
  // 通过比较当前聚类与历史地图（ikd-tree）来识别动态障碍物
  int k = 0;
  detections.clear();  // 清空检测结果
  deleted_indices.clear();  // 清空删除索引

  // 遍历每个聚类
  for (auto& getIndices: cluster_indices) {
    PointsPtr cluster(new Points);
    PointVector PointDelete;
    double avg_dist = 0;  // 平均距离
    Eigen::Vector3d cluster_center;  // 聚类中心
    cluster_center.setZero();
    std::vector<double> avg_buffer;  // 距离缓冲区

    // 遍历聚类中的每个点
    for (auto& index : getIndices.indices) {
      cluster->points.push_back(ClusterPoints->points[index]);

      // 在历史地图（ikd-tree）中搜索最近的3个邻居
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(ClusterPoints->points[index], 3, points_near, pointSearchSqDis);

      // 累加最近邻距离
      avg_dist += sqrt(pointSearchSqDis[0]);
      avg_buffer.push_back(sqrt(pointSearchSqDis[0]));
      cluster_center += Eigen::Vector3d(ClusterPoints->points[index].x, ClusterPoints->points[index].y, ClusterPoints->points[index].z);
    }

    // 计算聚类的平均距离和中心
    avg_dist = avg_dist/getIndices.indices.size();
    cluster_center = cluster_center/getIndices.indices.size();

    // 计算距离方差（归一化）
    double var_dist = 0;
    for (auto& dist : avg_buffer) {
      var_dist += (dist - avg_dist) * (dist - avg_dist) / (avg_dist * avg_dist);
    }
    var_dist = var_dist/getIndices.indices.size();

    // 检查该聚类是否与现有跟踪目标关联
    // 如果已经被跟踪且有一定速度，则认为是动态物体
    bool on_track = false;
    for (int i = 0; i < trackers.size(); ++i) {
      Eigen::Vector3d tracker_center = trackers[i]->pos();
      // 计算聚类中心与跟踪器位置的距离
      double dist = sqrt((cluster_center(0) - tracker_center(0)) * (cluster_center(0) - tracker_center(0)) +
                         (cluster_center(1) - tracker_center(1)) * (cluster_center(1) - tracker_center(1)) +
                         (cluster_center(2) - tracker_center(2)) * (cluster_center(2) - tracker_center(2)));
      // 判断条件：距离<0.5m，跟踪时长>3帧，速度>0.2m/s
      if (dist < 0.5 && trackers[i]->age > 3 && trackers[i]->vel().norm() > 0.2) {
        on_track = true;
        break;
      }
    }

    // 获取聚类的边界框
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cluster, minPt, maxPt);  // 计算最小和最大边界点

    // 计算聚类中心点
    PointType center_pt;
    center_pt.x = (maxPt.x + minPt.x)/2;
    center_pt.y = (maxPt.y + minPt.y)/2;
    center_pt.z = (maxPt.z + minPt.z)/2;

    // 计算聚类中心到机器人的距离（未使用）
    double radius_xy = (center_pt.x - md_->T[0]) * (center_pt.x - md_->T[0]) +
                       (center_pt.y - md_->T[1]) * (center_pt.y - md_->T[1]);
    double radius_z = (center_pt.z - md_->T[2]) * (center_pt.z - md_->T[2]);


    // ========== 动态障碍物判定条件 ==========
    // 条件1：平均距离大于阈值 且 小于5m 且 方差小于阈值（说明与历史地图差异大）
    // 条件2：或者已经被跟踪器跟踪（on_track为true）
    if (avg_dist > mp_->thresh_dist && avg_dist < 5.0 && var_dist < mp_->thresh_var || on_track) {
      // 创建检测到的障碍物状态
      ObjectState state;
      state.id = k;
      state.position = cluster_center;
      state.size << maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z;
      detections.push_back(state);
      deleted_indices.push_back(getIndices);

      // 将动态点云添加到输出
      for (auto& index : getIndices.indices) {
        dynamic_points->points.push_back(ClusterPoints->points[index]);
      }

      // 以下代码已注释：用于创建删除框，从静态地图中移除动态点
      // BoxPointType box;
      // box.vertex_min[0] = minPt.x-0.2; box.vertex_min[1] = minPt.y-0.2; box.vertex_min[2] = minPt.z-0.2;
      // box.vertex_max[0] = maxPt.x+0.2; box.vertex_max[1] = maxPt.y+0.2; box.vertex_max[2] = maxPt.z+0.2;
      // delete_boxes.push_back(box);
    }
  }

  // 发布动态点云
  sensor_msgs::PointCloud2 dynamic_pts;
  pcl::toROSMsg(*dynamic_points, dynamic_pts);
  dynamic_pts.header.frame_id = "world";
  cloudPub.publish(dynamic_pts);

  // ========== 多目标跟踪与EKF滤波 ==========
  // 首次检测时初始化跟踪器
  if(trackers.size() == 0) {
    for(int i = 0; i < detections.size(); ++i) {
      // 为每个检测创建一个EKF跟踪器（采样周期0.02s）
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
      ekfPtr->reset(detections[i].position, id);  // 初始化跟踪器位置和ID
      id++;
      trackers.push_back(ekfPtr);
      previous_p.push_back(detections[i].position);  // 保存前一帧位置
      Eigen::Vector3d v0(0,0,0);
      previous_v.push_back(v0);  // 初始化速度为0
    }
    return;
  }

  // 存储物体状态消息
  fapp_obj_state_msgs::ObjectsStates states;
  std::vector<std::pair<int, int>> matchedPairs;  // 存储匹配的检测-跟踪器对

  // ========== 数据关联：匹配检测与跟踪器 ==========
  for (int i = 0; i < detections.size(); ++i) {
    double min_dist = 1000000;
    int min_index = -1;
    Eigen::Vector3d det_pos = detections[i].position;

    // 对每个检测，寻找最近的跟踪器
    for (int j = 0; j < trackers.size(); ++j) {
      trackers[j]->age = trackers[j]->age + 1;  // 跟踪器年龄+1
      Eigen::Vector3d track_pos = trackers[j]->pos();
      double dist = (det_pos - track_pos).norm();  // 计算欧氏距离
      if (dist < min_dist) {
        min_dist = dist;
        min_index = j;
      }
    }

    // 如果最近距离小于0.8m，则认为匹配成功
    if (min_dist < 0.8) {
      // 计算检测速度（基于位置差分）
      Eigen::Vector3d vel_detect = (detections[i].position - previous_p[min_index]) / 0.02;
      // 更新EKF跟踪器（速度采用新旧速度的平均值）
      trackers[min_index]->update(detections[i].position, 0.5*vel_detect + 0.5*previous_v[min_index] );

      matchedPairs.push_back(std::make_pair(i, min_index));  // 记录匹配对

      // 创建并发布物体状态消息
      ObjectState state;
      state.position = trackers[min_index]->pos();
      state.velocity = trackers[min_index]->vel();
      fapp_obj_state_msgs::State statemsg;
      statemsg.header.stamp = ros::Time::now();
      statemsg.position.x = state.position[0]; statemsg.position.y = state.position[1]; statemsg.position.z = state.position[2];
      statemsg.velocity.x = state.velocity[0]; statemsg.velocity.y = state.velocity[1]; statemsg.velocity.z = state.velocity[2];
      statemsg.size.x = 0; statemsg.size.y = 0; statemsg.size.z = 0;
      states.states.push_back(statemsg);

    } else {
      // 如果没有匹配的跟踪器，创建新的跟踪器
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
      ekfPtr->reset(detections[i].position, id);
      id++;
      trackers.push_back(ekfPtr);
    }
  }
  statesPub.publish(states);  // 发布所有物体状态

  // 可视化匹配的检测和跟踪器
  visualizeFunction(matchedPairs);

  // ========== 跟踪器管理：删除长时间未更新的跟踪器 ==========
  for (auto it = trackers.begin(); it != trackers.end();) {
      // 如果跟踪器超过20帧未更新，则删除
      // age: 总帧数, update_num: 更新次数
      if ((*it)->age - (*it)->update_num > 20)
        it = trackers.erase(it);
      else
        it++;
  }

  // 更新前一帧位置和速度
  previous_p.clear();
  previous_v.clear();
  for (int i = 0; i < trackers.size(); ++i ) {
    previous_p.push_back(trackers[i]->pos());
    previous_v.push_back(trackers[i]->vel());
  }

  // 计算并输出跟踪阶段耗时
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Tracking Time Cost (ms)： " << compTime <<std::endl;
}

/**
 * @brief EKF预测定时器回调函数
 * @param e 定时器事件
 * 功能：定期对所有跟踪器进行EKF预测，以便在两次测量之间维持跟踪
 */
void MappingRos::ekfPredictCallback(const ros::TimerEvent& e) {
  if (trackers.size() == 0)
    return;

  // 对每个跟踪器执行EKF预测步骤
  for (int i = 0; i < trackers.size(); ++i) {
    double update_dt = (ros::Time::now() - trackers[i]->last_update_stamp_).toSec();  // 计算距上次更新的时间（未使用）
    trackers[i]->predict();  // 执行EKF预测
  }
}

/**
 * @brief 地图发布定时器回调函数
 * @param e 定时器事件
 * 功能：定期发布静态地图（当前代码主体已注释）
 */
void MappingRos::mapPubCallback(const ros::TimerEvent& e) {
  if (Remaining_Points->points.size() == 0)
    return;

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // 以下代码已注释：用于从剩余点云中删除动态物体区域，生成纯静态地图
  // KD_TREE<PointType>::Ptr delete_tree;
  // delete_tree.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
  // delete_tree->Build(Remaining_Points->points);
  // delete_tree->Delete_Point_Boxes(delete_boxes);  // 删除动态物体边界框内的点
  //
  // Points static_pts;
  // delete_tree->flatten(delete_tree->Root_Node, delete_tree->PCL_Storage, NOT_RECORD);
  // static_pts.points = delete_tree->PCL_Storage;
  // static_pts.width = static_pts.points.size();
  // static_pts.height = 1;
  // static_pts.is_dense = true;
  //
  // delete_boxes.clear();
  // sensor_msgs::PointCloud2 map_static;
  // pcl::toROSMsg(static_pts, map_static);
  // map_static.header.frame_id = "world";
  // staticMapPub.publish(map_static);  // 发布静态地图
}

/**
 * @brief 计算两个物体状态之间的IoU相似度
 * @param state1 第一个物体状态
 * @param state2 第二个物体状态
 * @return IoU值（基于距离的归一化相似度）
 * 注意：这里使用的是基于距离的近似IoU，而非真正的交并比
 */
double MappingRos::iou(ObjectState state1, ObjectState state2) {
  double distance = (state1.position - state2.position).norm();  // 计算两物体中心距离
  double iou = atan(distance)*2/M_PI;  // 使用arctan函数归一化到[0,1]区间
  return iou;
}

/**
 * @brief 可视化函数：显示检测到的物体边界框和运动方向箭头
 * @param pairs 匹配的检测-跟踪器对
 * 功能：在RViz中绘制3D边界框和速度箭头
 */
void MappingRos::visualizeFunction(const std::vector<std::pair<int, int>> pairs) {
  visualization_msgs::MarkerArray poses;  // 存储位姿箭头标记
  visualization_msgs::MarkerArray boxes;  // 存储边界框标记

  // 遍历所有匹配对
  for (auto pair : pairs) {
    int detectionIndex = pair.first;  // 检测索引
    int trackerIndex = pair.second;   // 跟踪器索引
    pcl::PointXYZ minPt, maxPt;

    // 以下过滤条件已注释
    // if (detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2 > 1.7 ||
    //     detections[detectionIndex].position[1] < -2.0 ||
    //     detections[detectionIndex].position[2] > 1.8)
    //   continue;

    // ========== 创建边界框可视化 ==========
    // 根据检测的位置和尺寸计算边界框顶点
    minPt.x = detections[detectionIndex].position[0] - detections[detectionIndex].size[0]/2;
    minPt.y = detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2;
    minPt.z = detections[detectionIndex].position[2] - detections[detectionIndex].size[2]/2;
    maxPt.x = detections[detectionIndex].position[0] + detections[detectionIndex].size[0]/2;
    maxPt.y = detections[detectionIndex].position[1] + detections[detectionIndex].size[1]/2;
    maxPt.z = detections[detectionIndex].position[2] + detections[detectionIndex].size[2]/2;

    // 创建边界框Marker
    visualization_msgs::Marker edgeMarker;
    edgeMarker.id = detectionIndex;
    edgeMarker.header.stamp = ros::Time::now();
    edgeMarker.header.frame_id = "world";
    edgeMarker.pose.orientation.w = 1.00;
    edgeMarker.lifetime = ros::Duration(0.1);  // 生命周期0.1秒
    edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;  // 线条类型
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;  // 橙色边界框
    edgeMarker.color.g = 0.50;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.80;  // 透明度80%
    edgeMarker.scale.x = 0.1;   // 线宽0.1m

    // 定义边界框的8个顶点
    geometry_msgs::Point point[8];
    point[0].x = minPt.x; point[0].y = maxPt.y; point[0].z = maxPt.z;
    point[1].x = minPt.x; point[1].y = minPt.y; point[1].z = maxPt.z;
    point[2].x = minPt.x; point[2].y = minPt.y; point[2].z = minPt.z;
    point[3].x = minPt.x; point[3].y = maxPt.y; point[3].z = minPt.z;
    point[4].x = maxPt.x; point[4].y = maxPt.y; point[4].z = minPt.z;
    point[5].x = maxPt.x; point[5].y = minPt.y; point[5].z = minPt.z;
    point[6].x = maxPt.x; point[6].y = minPt.y; point[6].z = maxPt.z;
    point[7].x = maxPt.x; point[7].y = maxPt.y; point[7].z = maxPt.z;

    // 按顺序连接顶点，形成立方体边框
    for (int l = 0; l < 8; l++) {
      edgeMarker.points.push_back(point[l]);
    }
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[3]);
    edgeMarker.points.push_back(point[2]);
    edgeMarker.points.push_back(point[5]);
    edgeMarker.points.push_back(point[6]);
    edgeMarker.points.push_back(point[1]);
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[7]);
    edgeMarker.points.push_back(point[4]);
    boxes.markers.push_back(edgeMarker);

    // ========== 创建速度箭头可视化 ==========
    visualization_msgs::Marker poseMarker;
    ObjectState state;
    state.position = trackers[trackerIndex]->pos();
    state.velocity = trackers[trackerIndex]->vel();
    poseMarker.id = trackerIndex+100;
    poseMarker.header.stamp = ros::Time::now();
    poseMarker.header.frame_id = "world";
    poseMarker.lifetime = ros::Duration(0.1);
    poseMarker.type = visualization_msgs::Marker::ARROW;  // 箭头类型
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.ns = "objectpose";
    poseMarker.color.r = 0.00;  // 绿色箭头
    poseMarker.color.g = 1.00;
    poseMarker.color.b = 0.00;
    poseMarker.color.a = 1.00;
    poseMarker.scale.x = 0.10;  // 箭头轴直径
    poseMarker.scale.y = 0.18;  // 箭头头部直径
    poseMarker.scale.z = 0.30;  // 箭头头部长度
    poseMarker.pose.orientation.w = 1.0;

    // 定义箭头的起点和终点
    geometry_msgs::Point arrow[2];
    arrow[0].x = state.position[0]; arrow[0].y = state.position[1]; arrow[0].z = state.position[2];  // 起点：物体位置
    // 终点：沿速度方向延伸1m（归一化速度向量）
    arrow[1].x = state.position[0] + 1.0*state.velocity[0]/state.velocity.norm();
    arrow[1].y = state.position[1] + 1.0*state.velocity[1]/state.velocity.norm();
    arrow[1].z = state.position[2] + 1.0*state.velocity[2]/state.velocity.norm();

    poseMarker.points.push_back(arrow[0]);
    poseMarker.points.push_back(arrow[1]);

    // 只显示速度大于0.01m/s的物体箭头
    if (state.velocity.norm() > 0.01)
      poses.markers.push_back(poseMarker);
  }

  // 发布可视化标记
  objectPosePub.publish(poses);
  edgePub.publish(boxes);
}

}
