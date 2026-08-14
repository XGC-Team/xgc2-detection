// KD树头文件 - 用于高效的空间搜索和点云聚类
// 基于nanoflann库实现快速的K近邻和半径搜索
#include <memory>
#include <nanoflann.hpp>  // 轻量级KD树库，用于快速最近邻搜索
#include <vector>

#include "point.h"

// kdtree命名空间 - 包含KD树相关的数据结构和算法
namespace kdtree {

/**
 * @brief KD树适配器结构体
 *
 * 用于将Point点云数据适配到nanoflann库的KD树接口
 * 提供nanoflann所需的数据访问接口，使其能够构建和查询KD树
 */
struct adaptor {
    std::vector<Point> m_points;  // 存储点云数据的向量

    /**
     * @brief 构造函数
     * @param points 输入的点云数据引用
     */
    explicit adaptor(std::vector<Point>& points)
        : m_points(points)
    {
    }

    /**
     * @brief 获取点云中点的数量
     * @return 点的总数
     *
     * nanoflann库要求的接口函数，用于确定KD树的大小
     */
    [[nodiscard]] inline size_t kdtree_get_point_count() const
    {
        return m_points.size();
    }

    /**
     * @brief 获取指定点的指定维度坐标
     * @param index 点的索引
     * @param dim 维度索引 (0=x, 1=y, 2=z)
     * @return 该点在指定维度的坐标值
     *
     * nanoflann库要求的接口函数，用于访问点的坐标数据
     */
    [[nodiscard]] inline float kdtree_get_pt(
        const size_t index, const size_t dim) const
    {
        switch (dim) {
        case 0:
            return m_points[index].m_xyz[0];  // 返回x坐标
        case 1:
            return m_points[index].m_xyz[1];  // 返回y坐标
        default:
            return m_points[index].m_xyz[2];  // 返回z坐标
        }
    }

    /**
     * @brief 获取点云的边界盒
     * @param bb 边界盒参数（未使用）
     * @return false 表示不使用边界盒优化
     *
     * nanoflann库的可选接口，此处返回false表示不使用边界盒
     */
    template <class BBOX> bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

/**
 * @brief 获取查询点的坐标
 * @param points 点云数据
 * @param index 点的索引
 * @return 包含x,y,z坐标的数组
 *
 * 将Point对象转换为std::array格式，用于KD树的查询操作
 */
std::array<float, 3> get_query_point(std::vector<Point>& points, size_t index)
{
    return std::array<float, 3>({ (float)points[index].m_xyz[0],
        (float)points[index].m_xyz[1], (float)points[index].m_xyz[2] });
}

/**
 * @brief DBSCAN密度聚类算法实现
 * @param points 待聚类的点云数据
 * @param eps 邻域半径（epsilon），定义点的邻域范围
 * @param min_pts 最小点数，一个点成为核心点所需的邻域内最小点数
 * @return 聚类结果，每个元素是一个簇，包含该簇中所有点的索引
 *
 * DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
 * 基于密度的空间聚类算法，能够发现任意形状的簇并识别噪声点
 * 使用KD树加速邻域搜索，时间复杂度为O(n log n)
 */
std::vector<std::vector<unsigned long>> dbscan(
    std::vector<Point>& points, float eps, int min_pts)
{
    eps *= eps;  // 转换为平方距离，避免半径搜索时的开方运算
    const auto adapt = adaptor(points);  // 创建KD树适配器
    // 定义KD树类型：使用L2距离（欧氏距离），3维空间
    using index_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, decltype(adapt)>, decltype(adapt),
        3>;

    // 构建KD树索引，参数10是叶子节点的最大点数
    index_t indexAdaptor(
        3, adapt, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    indexAdaptor.buildIndex();  // 构建KD树

    auto visited = std::vector<bool>(points.size());  // 标记点是否已被访问
    auto clusters = std::vector<std::vector<size_t>>();  // 存储所有的簇
    auto matches = std::vector<std::pair<size_t, float>>();  // 当前点的邻域点
    auto sub_matches = std::vector<std::pair<size_t, float>>();  // 邻域点的邻域点

    // 遍历所有点，尝试从每个未访问的点开始形成簇
    for (size_t i = 0; i < points.size(); i++) {
        if (visited[i])  // 跳过已访问的点
            continue;

        // 在eps半径内搜索邻近点
        indexAdaptor.radiusSearch(get_query_point(points, i).data(), eps,
            matches, nanoflann::SearchParams(32, 0.f, false));
        // 如果邻域内点数不足min_pts，则该点不是核心点，跳过
        if (matches.size() < static_cast<size_t>(min_pts))
            continue;
        visited[i] = true;  // 标记为已访问

        std::vector<size_t> cluster = { i };  // 创建新簇，并将当前点加入

        // 广度优先搜索扩展簇：处理所有邻域点
        while (!matches.empty()) {
            auto nb_idx = matches.back().first;  // 获取邻域点的索引
            matches.pop_back();  // 移除已处理的点
            if (visited[nb_idx])  // 如果该邻域点已被访问，跳过
                continue;
            visited[nb_idx] = true;  // 标记邻域点为已访问

            // 搜索邻域点的邻域
            indexAdaptor.radiusSearch(get_query_point(points, nb_idx).data(),
                eps, sub_matches, nanoflann::SearchParams(32, 0.f, false));

            // 如果邻域点也是核心点，将其邻域点加入待处理队列
            if (sub_matches.size() >= static_cast<size_t>(min_pts)) {
                std::copy(sub_matches.begin(), sub_matches.end(),
                    std::back_inserter(matches));
            }
            cluster.push_back(nb_idx);  // 将邻域点加入当前簇
        }
        clusters.emplace_back(std::move(cluster));  // 将完整的簇加入结果
    }
    return clusters;  // 返回所有簇
}

/**
 * @brief 点云聚类封装函数
 * @param sptr_points 待聚类的点云数据
 * @param E 邻域半径参数
 * @param N 最小点数参数
 * @return 聚类结果，每个元素是一个簇，包含该簇中所有点的索引
 *
 * 对DBSCAN算法的简单封装，提供更简洁的接口
 */
std::vector<std::vector<unsigned long>> cluster(
    std::vector<Point>& sptr_points, const float& E, const int& N)
{
    return kdtree::dbscan(sptr_points, E, N);
}
}  // namespace kdtree
