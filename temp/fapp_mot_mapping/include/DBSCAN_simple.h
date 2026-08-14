// DBSCAN_simple.h
// 简化版DBSCAN聚类算法实现
// DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
// 基于密度的空间聚类算法,能够发现任意形状的聚类,并能有效识别噪声点

#ifndef DBSCAN_H
#define DBSCAN_H

#include <pcl/point_types.h>  // PCL点云类型定义
#include <nanoflann.hpp>       // 高效KD树库,用于快速近邻搜索

// 点的处理状态定义
#define UN_PROCESSED 0  // 未处理状态
#define PROCESSING 1    // 处理中状态
#define PROCESSED 2     // 已处理状态

/**
 * @brief 比较两个点云聚类的大小
 * @param a 第一个点云索引集合
 * @param b 第二个点云索引集合
 * @return 如果a的点数少于b则返回true,用于升序排序
 */
inline bool comparePointClusters(const pcl::PointIndices &a, const pcl::PointIndices &b)
{
    return (a.indices.size() < b.indices.size());
}

/**
 * @brief 简化版DBSCAN点云聚类算法类
 * @tparam PointT 点云数据类型(如pcl::PointXYZ)
 *
 * DBSCAN算法核心思想:
 * 1. 核心点(Core Point): 在eps半径内至少有minPts个邻居点
 * 2. 边界点(Border Point): 不是核心点,但在某个核心点的邻域内
 * 3. 噪声点(Noise Point): 既不是核心点也不是边界点
 *
 * 算法流程:
 * 1. 遍历所有未处理的点
 * 2. 对每个点进行半径搜索,找到邻域内的所有点
 * 3. 如果邻居数量小于minPts,标记为噪声点
 * 4. 否则创建新聚类,将所有密度可达的点加入该聚类
 */
template <typename PointT>
class DBSCANSimpleCluster
{

public:
    // 类型定义
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;  // 点云智能指针类型
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;  // KD树智能指针类型
    /**
     * @brief 设置输入点云
     * @param cloud 待聚类的点云数据
     */
    virtual void setInputCloud(PointCloudPtr cloud)
    {
        input_cloud_ = cloud;
    }

    /**
     * @brief 设置搜索方法(使用PCL的KdTree)
     * @param tree KD树搜索结构
     */
    void setSearchMethod(KdTreePtr tree)
    {
             search_method_ = tree;
    }

    /**
     * @brief 设置搜索方法(使用nanoflann的KdTree)
     * 创建并初始化nanoflann的KD树索引结构,用于高效的近邻搜索
     */
    void setSearchMethod()
    {
        pcPtr_ = std::make_shared<Obs>();  // 创建点云适配器
        pcPtr_->pts = input_cloud_;        // 设置点云数据
        kdPtr_ = std::make_shared<my_kd_tree_t>(3, *pcPtr_);  // 创建3维KD树
        kdPtr_->buildIndex();              // 构建KD树索引
    }

    /**
     * @brief 使用nanoflann执行DBSCAN聚类提取
     * @param cluster_indices 输出参数,存储所有聚类的点索引集合
     *
     * 核心DBSCAN算法实现:
     * - 使用nanoflann进行半径搜索以提高效率
     * - 通过种子队列扩展聚类区域
     * - 自动过滤噪声点和不符合大小要求的聚类
     */
    void extractNano(std::vector<pcl::PointIndices> &cluster_indices)
    {
         std::vector<std::pair<long unsigned int, double>> ret_matches;  // nanoflann搜索结果:存储点索引和距离

        std::vector<bool> is_noise(input_cloud_->points.size(), false);      // 标记噪声点
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);   // 存储每个点的处理状态
        // 遍历点云中的每一个点
        for (size_t i = 0; i < input_cloud_->points.size(); i++)
        {
            // 跳过已处理的点
            if (types[i] == PROCESSED)
            {
                continue;
            }

            // 在eps半径内搜索邻近点
            size_t nn_size = radiusSearchNano(i, eps_, ret_matches);

            // 如果邻居数量小于minPts,标记为噪声点
            if (nn_size < minPts_)
            {
                is_noise[i] = true;
                continue;
            }

            // 当前点是核心点,创建新聚类
            std::vector<uint32_t> seed_queue;  // 种子队列,用于扩展聚类
            seed_queue.push_back(i);           // 将核心点加入队列
            types[i] = PROCESSED;              // 标记为已处理

            // 将所有邻近点加入种子队列
            for (size_t j = 0; j < nn_size; j++)
            {
                if (ret_matches[j].first  != i)
                {
                    seed_queue.push_back(ret_matches[j].first);
                    types[ret_matches[j].first] = PROCESSING;  // 标记为处理中
                }
            }
            // 扩展聚类:处理种子队列中的所有点
            size_t sq_idx = 1;  // 从索引1开始(索引0是核心点本身)
            while (sq_idx < seed_queue.size())
            {
                int cloud_index = seed_queue[sq_idx];

                // 如果是噪声点或已处理,跳过
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
                {
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue;  // 不需要检查邻居
                }

                // 对当前点进行半径搜索
                nn_size = radiusSearchNano(cloud_index, eps_, ret_matches);

                // 如果当前点也是核心点,将其未处理的邻居加入种子队列
                if (nn_size >= minPts_)
                {
                    for (size_t j = 0; j < nn_size; j++)
                    {
                        if (types[ret_matches[j].first] == UN_PROCESSED)
                        {
                            seed_queue.push_back(ret_matches[j].first);
                            types[ret_matches[j].first] = PROCESSING;
                        }
                    }
                }

                types[cloud_index] = PROCESSED;  // 标记当前点已处理
                sq_idx++;
            }
            // 检查聚类大小是否在有效范围内
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size() <= max_pts_per_cluster_)
            {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                // 将种子队列中的索引复制到结果中
                for (size_t j = 0; j < seed_queue.size(); ++j)
                {
                    r.indices[j] = seed_queue[j];
                }
                // 排序并去重(理论上不需要,但保险起见)
                std::sort(r.indices.begin(), r.indices.end());
                r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

                r.header = input_cloud_->header;  // 保留点云头信息
                cluster_indices.push_back(r);     // 将聚类添加到结果集
            }
        } // 遍历完所有点

        // 按聚类大小降序排序(最大的聚类在前)
        std::sort(cluster_indices.rbegin(), cluster_indices.rend(), comparePointClusters);
    }

    /**
     * @brief 使用PCL KdTree执行DBSCAN聚类提取(已注释的备用实现)
     *
     * 这是使用PCL库KdTree的原始实现版本,功能与extractNano相同
     * 当前被注释掉,保留作为参考
     */
    // void extract(std::vector<pcl::PointIndices> &cluster_indices)
    // {
    //     std::vector<int> nn_indices;
    //     std::vector<float> nn_distances;
    //     std::vector<bool> is_noise(input_cloud_->points.size(), false);
    //     std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
    //     for (size_t i = 0; i < input_cloud_->points.size(); i++)
    //     {
    //         if (types[i] == PROCESSED)
    //         {
    //             continue;
    //         }
    //         int nn_size = radiusSearch(i, eps_, nn_indices, nn_distances);
    //         if (nn_size < minPts_)
    //         {
    //             is_noise[i] = true;
    //             continue;
    //         }

    //         std::vector<int> seed_queue;
    //         seed_queue.push_back(i);
    //         types[i] = PROCESSED;

    //         for (int j = 0; j < nn_size; j++)
    //         {
    //             if (nn_indices[j] != i)
    //             {
    //                 seed_queue.push_back(nn_indices[j]);
    //                 types[nn_indices[j]] = PROCESSING;
    //             }
    //         } // for every point near the chosen core point.
    //         size_t sq_idx = 1;
    //         while (sq_idx < seed_queue.size())
    //         {
    //             int cloud_index = seed_queue[sq_idx];
    //             if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
    //             {
    //                 // seed_queue.push_back(cloud_index);
    //                 types[cloud_index] = PROCESSED;
    //                 sq_idx++;
    //                 continue; // no need to check neighbors.
    //             }
    //             nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
    //             if (nn_size >= minPts_)
    //             {
    //                 for (int j = 0; j < nn_size; j++)
    //                 {
    //                     if (types[nn_indices[j]] == UN_PROCESSED)
    //                     {

    //                         seed_queue.push_back(nn_indices[j]);
    //                         types[nn_indices[j]] = PROCESSING;
    //                     }
    //                 }
    //             }

    //             types[cloud_index] = PROCESSED;
    //             sq_idx++;
    //         }
    //         if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size() <= max_pts_per_cluster_)
    //         {
    //             pcl::PointIndices r;
    //             r.indices.resize(seed_queue.size());
    //             for (int j = 0; j < seed_queue.size(); ++j)
    //             {
    //                 r.indices[j] = seed_queue[j];
    //             }
    //             // These two lines should not be needed: (can anyone confirm?) -FF
    //             std::sort(r.indices.begin(), r.indices.end());
    //             r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

    //             r.header = input_cloud_->header;
    //             cluster_indices.push_back(r); // We could avoid a copy by working directly in the vector
    //         }
    //     } // for every point in input cloud
    //     std::sort(cluster_indices.rbegin(), cluster_indices.rend(), comparePointClusters);
    // }

    /**
     * @brief 设置聚类容差(eps参数)
     * @param tolerance 邻域半径,用于定义点的邻近关系
     */
    void setClusterTolerance(double tolerance)
    {
        eps_ = tolerance;
    }

    /**
     * @brief 设置最小聚类大小
     * @param min_cluster_size 聚类中最少点数,小于此值的聚类将被过滤
     */
    void setMinClusterSize(int min_cluster_size)
    {
        min_pts_per_cluster_ = min_cluster_size;
    }

    /**
     * @brief 设置最大聚类大小
     * @param max_cluster_size 聚类中最多点数,大于此值的聚类将被过滤
     */
    void setMaxClusterSize(int max_cluster_size)
    {
        max_pts_per_cluster_ = max_cluster_size;
    }

    /**
     * @brief 设置核心点的最小邻居数(minPts参数)
     * @param core_point_min_pts 成为核心点所需的最小邻居数量
     */
    void setCorePointMinPts(int core_point_min_pts)
    {
        minPts_ = core_point_min_pts;
    }

protected:
    // 成员变量
    PointCloudPtr input_cloud_;  // 输入点云数据

    // DBSCAN算法参数
    double eps_{0.0};                                               // 邻域半径epsilon
    size_t minPts_{1};                                              // 核心点的最小邻居数(不包括点本身)
    size_t min_pts_per_cluster_{1};                                 // 有效聚类的最小点数
    size_t max_pts_per_cluster_{std::numeric_limits<int>::max()};  // 有效聚类的最大点数

    KdTreePtr search_method_;  // PCL KdTree搜索方法

    /**
     * @brief 使用nanoflann执行半径搜索
     * @param index 查询点在点云中的索引
     * @param radius 搜索半径
     * @param ret_matches 输出参数,存储搜索到的点索引和距离对
     * @return 找到的邻居点数量
     */
    size_t radiusSearchNano(
        int index, double radius, std::vector<std::pair<long unsigned int, double>>& ret_matches)
        {
        // 构建查询点坐标
        check_pt = {input_cloud_->points[index].x,input_cloud_->points[index].y,input_cloud_->points[index].z};
        // 使用nanoflann的KD树执行半径搜索
        return kdPtr_->radiusSearch(
            check_pt.data(), radius, ret_matches, params);
        }

    /**
     * @brief 暴力半径搜索实现(备用方法)
     * @param index 查询点在点云中的索引
     * @param radius 搜索半径
     * @param k_indices 输出参数,存储找到的点索引
     * @param k_sqr_distances 输出参数,存储对应的欧式距离
     * @return 找到的邻居点数量
     *
     * 注意:这是一个简单的线性搜索实现,效率较低,适用于小规模点云
     * 对于大规模点云应使用KD树加速
     */
    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();
        k_indices.push_back(index);       // 首先加入查询点本身
        k_sqr_distances.push_back(0);     // 到自身的距离为0

        int size = input_cloud_->points.size();
        double radius_square = radius * radius;  // 预计算半径平方以避免开方运算

        // 遍历所有点
        for (int i = 0; i < size; i++)
        {
            if (i == index)
            {
                continue;  // 跳过查询点本身
            }

            // 计算欧式距离的平方
            double distance_x = input_cloud_->points[i].x - input_cloud_->points[index].x;
            double distance_y = input_cloud_->points[i].y - input_cloud_->points[index].y;
            double distance_z = input_cloud_->points[i].z - input_cloud_->points[index].z;
            double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;

            // 如果距离小于等于半径,加入结果
            if (distance_square <= radius_square)
            {
                k_indices.push_back(i);
                k_sqr_distances.push_back(std::sqrt(distance_square));  // 存储实际距离
            }
        }
        return k_indices.size();
    }
private:
    // nanoflann相关私有成员
    std::vector<double> check_pt;  // 用于存储查询点坐标的临时变量

    /**
     * @brief 点云数据适配器结构体
     *
     * 为nanoflann库提供访问PCL点云数据的接口
     * nanoflann需要通过此适配器来访问点云数据结构
     */
    struct Obs
    {
        PointCloudPtr pts;  // PCL点云数据指针

        /**
         * @brief 返回点云中的点数量
         * @return 点的总数
         */
        inline size_t kdtree_get_point_count() const { return pts->points.size(); }

        /**
         * @brief 获取指定点的指定维度坐标
         * @param idx 点的索引
         * @param dim 维度索引(0=x, 1=y, 2=z)
         * @return 对应维度的坐标值
         */
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            if (dim == 0)
                return pts->points[idx].x;
            else if (dim == 1)
                return pts->points[idx].y;
            else
                return pts->points[idx].z;
        }

        /**
         * @brief 获取边界框(未实现)
         * @return 始终返回false,表示不使用边界框优化
         */
        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
    };

    /**
     * @brief nanoflann KD树类型定义
     *
     * 使用L2范数(欧式距离)的单索引KD树
     * 参数说明:
     * - L2_Simple_Adaptor: 使用L2范数计算距离
     * - Obs: 点云数据适配器
     * - 3: 三维空间
     */
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, Obs>,
        Obs, 3 /* dim */>
        my_kd_tree_t;

    std::shared_ptr<Obs> pcPtr_;           // 点云适配器智能指针
    std::shared_ptr<my_kd_tree_t> kdPtr_;  // KD树智能指针
    nanoflann::SearchParams params;        // nanoflann搜索参数
}; // class DBSCANSimpleCluster

#endif // DBSCAN_H