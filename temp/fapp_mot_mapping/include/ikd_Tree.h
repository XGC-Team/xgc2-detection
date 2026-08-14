/**
 * @file ikd_Tree.h
 * @author Guanhua Wang (you@domain.com)
 * @brief a header only version of ikd-tree, C++ style.
 * @version 0.1
 * @date 2022-07-06
 *
 * @copyright Copyright (c) 2022
 *
 * Description: ikd-Tree: an incremental k-d tree for robotic applications
 * Author: Yixi Cai
 * email: yixicai@connect.hku.hk
 *
 * 说明: ikd-Tree（增量式k-d树）是一种针对机器人应用优化的动态k-d树数据结构
 * 主要特性:
 * 1. 支持增量式点云插入和删除
 * 2. 自动平衡树结构以保证查询效率
 * 3. 支持多线程重建优化
 * 4. 支持降采样功能
 * 5. 高效的k近邻搜索、半径搜索和box搜索
**/

#ifndef IKD_TREE_H_
#define IKD_TREE_H_

#include <cstdio>
#include <queue>
#include <chrono>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <memory>
#include <Eigen/Core>
#include <pthread.h> // Linux/Unix系统接口，c++<std::thread>不能替代之。
#include <unistd.h> // Linux/Unix系统中内置头文件,包含一些系统服务函数接口。

// 浮点数比较的精度阈值
#define EPSS 1e-6
// 最小不平衡树大小阈值，小于此值的树不进行重平衡操作
#define Minimal_Unbalanced_Tree_Size 10
// 多线程重建点数阈值，当需要重建的子树包含的点数超过此值时，启用额外线程处理
#define Multi_Thread_Rebuild_Point_Num 1500
// 降采样开关
#define DOWNSAMPLE_SWITCH true
// 强制重建百分比阈值，当删除点数占总点数的比例超过此值时触发强制重建
#define ForceRebuildPercentage 0.2
// 操作队列的最大长度
#define Q_LEN 1000000

using namespace std;

/**
 * @brief ikd-Tree中使用的点类型定义
 *
 * 该结构体定义了三维空间中的点，包含x、y、z三个坐标分量
 * 使用float类型以节省内存并提高性能
 */
struct ikdTree_PointType
{
    float x, y, z;  // 点的三维坐标

    /**
     * @brief 构造函数，初始化三维点
     * @param px x坐标，默认为0.0
     * @param py y坐标，默认为0.0
     * @param pz z坐标，默认为0.0
     *
     * 注意：这里使用值传递而非const引用，因为float类型本身很小(4字节)
     * 值传递的开销不大，反而避免了引用传递的间接访问开销
     */
    ikdTree_PointType (float px = 0.0f, float py = 0.0f, float pz = 0.0f){
        x = px;
        y = py;
        z = pz;
    }
};

/**
 * @brief 三维空间中的边界框类型定义
 *
 * 边界框由两个顶点定义：最小顶点和最大顶点
 * 用于区域查询、删除和碰撞检测等操作
 */
struct BoxPointType{
    float vertex_min[3];  // 边界框的最小顶点坐标 [x_min, y_min, z_min]
    float vertex_max[3];  // 边界框的最大顶点坐标 [x_max, y_max, z_max]
};

/**
 * @brief ikd-Tree支持的操作类型枚举
 *
 * 定义了树结构可以执行的所有操作类型
 * 这些操作会被记录在操作日志中，用于多线程重建时回放
 */
enum operation_set {
    ADD_POINT,          // 添加单个点
    DELETE_POINT,       // 删除单个点
    DELETE_BOX,         // 删除边界框内的所有点
    ADD_BOX,            // 添加边界框内的所有点
    DOWNSAMPLE_DELETE,  // 降采样删除操作
    PUSH_DOWN           // 将删除标记向下传播到子节点
};

/**
 * @brief 删除点的存储方式枚举
 *
 * 用于控制在删除操作中如何记录被删除的点
 * 不同的存储方式适用于不同的应用场景
 */
enum delete_point_storage_set {
    NOT_RECORD,         // 不记录被删除的点
    DELETE_POINTS_REC,  // 记录被删除的点到删除点列表中
    MULTI_THREAD_REC    // 在多线程模式下记录被删除的点
};

/**
 * @brief 手动实现的循环队列类
 * @tparam T 队列中存储的元素类型
 *
 * 该类实现了一个基于数组的循环队列，用于存储操作日志
 * 相比STL的queue，这个实现更加轻量级且性能可控
 * 主要用于多线程重建时记录操作历史
 */
template <typename T>
class MANUAL_Q{
    private:
        int head = 0;      // 队列头部索引
        int tail = 0;      // 队列尾部索引
        int counter = 0;   // 队列中元素的数量
        T q[Q_LEN];        // 存储队列元素的数组
        bool is_empty;     // 队列是否为空的标志

    public:
        void pop();        // 弹出队首元素
        T front();         // 返回队首元素（不删除）
        T back();          // 返回队尾元素（不删除）
        void clear();      // 清空队列
        void push(T op);   // 向队尾添加元素
        bool empty();      // 判断队列是否为空
        int size();        // 返回队列中元素的数量
};

/**
 * @brief ikd-Tree核心类
 * @tparam PointType 点的数据类型（如ikdTree_PointType或PCL点云类型）
 *
 * 这是ikd-Tree的主类，实现了增量式k-d树的所有功能
 * 主要特性：
 * 1. 动态插入和删除点
 * 2. 自动平衡维护
 * 3. 多线程重建支持
 * 4. k近邻搜索、半径搜索、区域搜索
 * 5. 降采样功能
 */
template<typename PointType>
class KD_TREE {
  public:
    // 使用Eigen对齐分配器的点向量类型，确保SSE/AVX等SIMD指令的正确对齐
    using PointVector = vector<PointType, Eigen::aligned_allocator<PointType>>;
    // 智能指针类型，便于管理KD_TREE对象
    using Ptr = shared_ptr<KD_TREE<PointType>>;

    /**
     * @brief k-d树节点结构体
     *
     * 每个节点包含一个点、分割轴信息、子树统计信息和各种状态标志
     * 支持懒删除（lazy deletion）和增量式更新
     */
    struct KD_TREE_NODE{
        PointType point;                            // 该节点存储的数据点
        uint8_t division_axis;                      // 分割轴（0=x轴, 1=y轴, 2=z轴）
        int TreeSize = 1;                           // 以该节点为根的子树的总节点数
        int invalid_point_num = 0;                  // 以该节点为根的子树中被标记为删除的点的数量
        int down_del_num = 0;                       // 降采样删除的点数
        bool point_deleted = false;                 // 该节点的点是否被删除
        bool tree_deleted = false;                  // 以该节点为根的整个子树是否被标记为删除
        bool point_downsample_deleted = false;      // 该节点的点是否因降采样而被删除
        bool tree_downsample_deleted = false;       // 以该节点为根的子树是否因降采样而被删除
        bool need_push_down_to_left = false;        // 是否需要将删除标记向左子树传播
        bool need_push_down_to_right = false;       // 是否需要将删除标记向右子树传播
        bool working_flag = false;                  // 工作标志，用于多线程同步
        float radius_sq;                            // 半径的平方，用于半径搜索优化
        pthread_mutex_t push_down_mutex_lock;       // 用于push_down操作的互斥锁
        float node_range_x[2], node_range_y[2], node_range_z[2];  // 该子树的包络盒范围 [min, max]
        KD_TREE_NODE *left_son_ptr = nullptr;       // 左子树指针
        KD_TREE_NODE *right_son_ptr = nullptr;      // 右子树指针
        KD_TREE_NODE *father_ptr = nullptr;         // 父节点指针
        // 用于论文数据记录的参数
        float alpha_del;                            // 删除不平衡度：invalid_point_num / TreeSize
        float alpha_bal;                            // 平衡因子：较小子树size / 较大子树size
    };

    /**
     * @brief 操作日志类型
     *
     * 记录对树的操作，用于多线程重建时回放操作
     * 在重建过程中，新的操作会被记录到日志中，重建完成后再应用这些操作
     */
    struct Operation_Logger_Type{
        PointType point;                            // 操作涉及的点
        BoxPointType boxpoint;                      // 操作涉及的边界框
        bool tree_deleted, tree_downsample_deleted; // 树的删除和降采样删除标志
        operation_set op;                           // 操作类型
    };

    /**
     * @brief 带距离的点类型，用于k近邻搜索
     *
     * 该结构体将点和其到查询点的距离绑定在一起
     * 实现了比较运算符，用于在优先队列中排序
     */
    struct PointType_CMP{
        PointType point;   // 候选点
        float dist = 0.0;  // 该点到查询点的距离

        /**
         * @brief 构造函数
         * @param p 点数据
         * @param d 距离值，默认为无穷大
         */
        PointType_CMP (PointType p = PointType(), float d = INFINITY){
            this->point = p;
            this->dist = d;
        };

        /**
         * @brief 小于运算符重载，用于堆排序
         * @param a 比较对象
         * @return 如果距离相等则比较x坐标，否则比较距离
         */
        bool operator < (const PointType_CMP &a)const{
            if (fabs(dist - a.dist) < 1e-10) return point.x < a.point.x;
            else return dist < a.dist;
        }
    };

    /**
     * @brief 手动实现的最大堆类
     *
     * 用于k近邻搜索中维护当前k个最近的候选点
     * 使用最大堆，堆顶是当前k个点中距离最大的点
     * 相比STL的priority_queue，这个实现更加轻量级且可控
     */
    class MANUAL_HEAP{
        public:
            /**
             * @brief 构造函数
             * @param max_capacity 堆的最大容量，默认100
             */
            MANUAL_HEAP(int max_capacity = 100){
                cap = max_capacity;
                heap = new PointType_CMP[max_capacity];
                heap_size = 0;
            }

            /**
             * @brief 析构函数，释放堆内存
             */
            ~MANUAL_HEAP(){ delete[] heap;}

            /**
             * @brief 弹出堆顶元素（距离最大的点）
             */
            void pop(){
                if (heap_size == 0) return;
                heap[0] = heap[heap_size-1];
                heap_size--;
                MoveDown(0);
                return;
            }

            /**
             * @brief 获取堆顶元素（不删除）
             * @return 堆顶的点和距离
             */
            PointType_CMP top(){ return heap[0];}

            /**
             * @brief 向堆中插入一个点
             * @param point 待插入的点和距离
             */
            void push(PointType_CMP point){
                if (heap_size >= cap) return;
                heap[heap_size] = point;
                FloatUp(heap_size);
                heap_size++;
                return;
            }

            /**
             * @brief 获取堆中元素的数量
             * @return 堆的大小
             */
            int size(){ return heap_size;}

            /**
             * @brief 清空堆
             */
            void clear(){ heap_size = 0;}

        private:
            int heap_size = 0;        // 堆中当前元素的数量
            int cap = 0;              // 堆的最大容量
            PointType_CMP * heap;     // 存储堆元素的数组

            /**
             * @brief 向下调整堆（用于删除操作后的堆调整）
             * @param heap_index 需要调整的节点索引
             */
            void MoveDown(int heap_index){
                int l = heap_index * 2 + 1;  // 左孩子索引
                PointType_CMP tmp = heap[heap_index];
                while (l < heap_size){
                    // 如果右孩子存在且大于左孩子，选择右孩子
                    if (l + 1 < heap_size && heap[l] < heap[l+1]) l++;
                    // 如果当前节点小于孩子节点，需要交换
                    if (tmp < heap[l]){
                        heap[heap_index] = heap[l];
                        heap_index = l;
                        l = heap_index * 2 + 1;
                    } else break;
                }
                heap[heap_index] = tmp;
                return;
            }

            /**
             * @brief 向上调整堆（用于插入操作后的堆调整）
             * @param heap_index 需要调整的节点索引
             *
             * 将新插入的元素向上移动到合适的位置，保持最大堆性质
             */
            void FloatUp(int heap_index){
                int ancestor = (heap_index-1)/2;  // 父节点索引
                PointType_CMP tmp = heap[heap_index];
                while (heap_index > 0){
                    // 如果父节点小于当前节点，需要交换
                    if (heap[ancestor] < tmp){
                        heap[heap_index] = heap[ancestor];
                        heap_index = ancestor;
                        ancestor = (heap_index-1)/2;
                    } else break;
                }
                heap[heap_index] = tmp;
                return;
            }

    };    

  private:
    // ========== 多线程树重建相关成员变量 ==========
    bool termination_flag = false;           // 线程终止标志，用于通知重建线程退出
    bool rebuild_flag = false;               // 重建标志，指示是否正在进行重建
    pthread_t rebuild_thread;                // 重建线程句柄
    // 各种互斥锁，用于保护共享资源
    pthread_mutex_t termination_flag_mutex_lock;      // 保护termination_flag的互斥锁
    pthread_mutex_t rebuild_ptr_mutex_lock;           // 保护Rebuild_Ptr的互斥锁
    pthread_mutex_t working_flag_mutex;               // 保护节点working_flag的互斥锁
    pthread_mutex_t search_flag_mutex;                // 保护搜索操作的互斥锁
    pthread_mutex_t rebuild_logger_mutex_lock;        // 保护操作日志的互斥锁
    pthread_mutex_t points_deleted_rebuild_mutex_lock;// 保护删除点列表的互斥锁
    MANUAL_Q<Operation_Logger_Type> Rebuild_Logger;   // 操作日志队列，记录重建期间的操作
    PointVector Rebuild_PCL_Storage;         // 重建时临时存储点云数据
    KD_TREE_NODE ** Rebuild_Ptr = nullptr;   // 指向需要重建的子树根节点的指针的指针
    int search_mutex_counter = 0;            // 搜索互斥计数器，用于读写锁实现

    // 多线程相关私有方法
    static void * multi_thread_ptr(void *arg);  // 重建线程的入口函数（静态方法）
    void multi_thread_rebuild();                // 多线程重建的实际执行函数
    void start_thread();                        // 启动重建线程
    void stop_thread();                         // 停止重建线程
    void run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation);  // 执行记录的操作

    // ========== k-d树核心功能相关成员变量 ==========
    int Treesize_tmp = 0, Validnum_tmp = 0;    // 临时变量：树大小和有效节点数
    float alpha_bal_tmp = 0.5;                 // 临时变量：平衡因子
    float alpha_del_tmp = 0.0;                 // 临时变量：删除因子
    float delete_criterion_param = 0.5f;       // 删除不平衡阈值参数，当alpha_del超过此值时触发重建
    float balance_criterion_param = 0.7f;      // 平衡不平衡阈值参数，当alpha_bal低于此值时触发重建
    float downsample_size = 0.2f;              // 降采样的体素大小
    bool Delete_Storage_Disabled = false;      // 是否禁用删除点存储
    KD_TREE_NODE * STATIC_ROOT_NODE = nullptr; // 静态根节点，树结构的基础，仅在初始构建或完全重建时改变

    // 存储相关的点集合
    PointVector Points_deleted;              // 存储被删除的点
    PointVector Downsample_Storage;          // 降采样临时存储
    PointVector Multithread_Points_deleted;  // 多线程模式下被删除的点

    // ========== k-d树核心算法私有方法 ==========

    /**
     * @brief 初始化树节点
     * @param root 要初始化的节点指针
     */
    void InitTreeNode(KD_TREE_NODE * root);

    /**
     * @brief 测试锁状态（用于调试）
     * @param root 要测试的节点
     */
    void Test_Lock_States(KD_TREE_NODE *root);

    /**
     * @brief 构建k-d树
     * @param root 根节点指针的指针
     * @param l 点集的左边界索引
     * @param r 点集的右边界索引
     * @param Storage 存储点的向量
     */
    void BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage);

    /**
     * @brief 重建子树
     * @param root 要重建的子树根节点指针的指针
     */
    void Rebuild(KD_TREE_NODE ** root);

    /**
     * @brief 按区域删除点
     * @param root 根节点指针的指针
     * @param boxpoint 边界框
     * @param allow_rebuild 是否允许重建
     * @param is_downsample 是否为降采样删除
     * @return 删除的点数
     */
    int Delete_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);

    /**
     * @brief 删除单个点
     * @param root 根节点指针的指针
     * @param point 要删除的点
     * @param allow_rebuild 是否允许重建
     */
    void Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild);

    /**
     * @brief 添加单个点
     * @param root 根节点指针的指针
     * @param point 要添加的点
     * @param allow_rebuild 是否允许重建
     * @param father_axis 父节点的分割轴
     */
    void Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild, int father_axis);

    /**
     * @brief 按区域添加点
     * @param root 根节点指针的指针
     * @param boxpoint 边界框
     * @param allow_rebuild 是否允许重建
     */
    void Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild);

    /**
     * @brief k近邻搜索核心递归函数
     * @param root 当前搜索的子树根节点
     * @param k_nearest 要查找的最近邻点数k
     * @param point 查询点
     * @param q 存储候选点的最大堆
     * @param max_dist 最大搜索距离
     */
    void Search(KD_TREE_NODE * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist);

    /**
     * @brief 区域搜索
     * @param root 当前搜索的子树根节点
     * @param boxpoint 查询的边界框
     * @param Storage 存储搜索结果的向量
     */
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);

    /**
     * @brief 半径搜索
     * @param root 当前搜索的子树根节点
     * @param point 查询点
     * @param radius 搜索半径
     * @param Storage 存储搜索结果的向量
     */
    void Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage);

    /**
     * @brief 检查节点是否满足重建标准
     * @param root 要检查的节点
     * @return 如果需要重建返回true，否则返回false
     */
    bool Criterion_Check(KD_TREE_NODE * root);

    /**
     * @brief 将删除标记向下推送到子节点
     * @param root 要执行push down的节点
     */
    void Push_Down(KD_TREE_NODE * root);

    /**
     * @brief 更新节点的统计信息（TreeSize、invalid_point_num等）
     * @param root 要更新的节点
     */
    void Update(KD_TREE_NODE * root);

    /**
     * @brief 删除树节点并释放内存
     * @param root 要删除的子树根节点指针的指针
     */
    void delete_tree_nodes(KD_TREE_NODE ** root);

    /**
     * @brief 对子树进行降采样
     * @param root 要降采样的子树根节点指针的指针
     */
    void downsample(KD_TREE_NODE ** root);

    /**
     * @brief 判断两个点是否相同
     * @param a 点a
     * @param b 点b
     * @return 如果两点相同返回true，否则返回false
     */
    bool same_point(PointType a, PointType b);

    /**
     * @brief 计算两点之间的欧氏距离
     * @param a 点a
     * @param b 点b
     * @return 两点之间的欧氏距离
     */
    float calc_dist(PointType a, PointType b);

    /**
     * @brief 计算点到节点包络盒的最小距离
     * @param node 树节点
     * @param point 查询点
     * @return 点到节点包络盒的最小距离
     */
    float calc_box_dist(KD_TREE_NODE * node, PointType point);

    // 点比较函数，用于按不同轴排序
    static bool point_cmp_x(PointType a, PointType b);  // 按x坐标比较
    static bool point_cmp_y(PointType a, PointType b);  // 按y坐标比较
    static bool point_cmp_z(PointType a, PointType b);  // 按z坐标比较 

  public:
    // ========== 公有成员变量 ==========
    PointVector PCL_Storage;              // 点云存储，用于初始构建和重建
    KD_TREE_NODE * Root_Node = nullptr;   // 树的根节点指针
    int max_queue_size = 0;               // 最大队列大小（用于统计）

    // ========== 构造函数和析构函数 ==========

    /**
     * @brief 构造函数
     * @param delete_param 删除不平衡阈值，默认0.5
     * @param balance_param 平衡不平衡阈值，默认0.6
     * @param box_length 降采样体素大小，默认0.2
     *
     * 创建一个新的ikd-Tree实例并初始化参数
     */
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);

    /**
     * @brief 析构函数
     *
     * 释放树的所有节点并清理多线程资源
     */
    ~KD_TREE();

    // ========== 参数设置方法 ==========

    /**
     * @brief 设置删除不平衡阈值参数
     * @param delete_param 删除不平衡阈值（0-1之间）
     *
     * 当节点的删除比例超过此阈值时，会触发重建
     */
    void Set_delete_criterion_param(float delete_param);

    /**
     * @brief 设置平衡不平衡阈值参数
     * @param balance_param 平衡不平衡阈值（0-1之间）
     *
     * 当节点的平衡因子低于此阈值时，会触发重建
     */
    void Set_balance_criterion_param(float balance_param);

    /**
     * @brief 设置降采样参数
     * @param box_length 降采样体素的边长
     *
     * 在每个体素内只保留一个点
     */
    void set_downsample_param(float box_length);

    /**
     * @brief 初始化k-d树
     * @param delete_param 删除不平衡阈值，默认0.5
     * @param balance_param 平衡不平衡阈值，默认0.7
     * @param box_length 降采样体素大小，默认0.2
     *
     * 重新初始化树的参数，会清空现有树
     */
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);

    // ========== 查询方法 ==========

    /**
     * @brief 获取树中的总节点数
     * @return 树中的节点总数
     */
    int size();

    /**
     * @brief 获取树中有效节点的数量
     * @return 未被删除的节点数量
     */
    int validnum();

    /**
     * @brief 获取根节点的平衡因子和删除因子
     * @param alpha_bal 输出参数：平衡因子
     * @param alpha_del 输出参数：删除因子
     */
    void root_alpha(float &alpha_bal, float &alpha_del);

    /**
     * @brief 获取树的边界范围
     * @return 包含整棵树的最小边界框
     */
    BoxPointType tree_range();

    // ========== 构建方法 ==========

    /**
     * @brief 从点云构建k-d树
     * @param point_cloud 输入点云
     *
     * 从给定的点云构建一个新的k-d树，会替换现有的树
     */
    void Build(PointVector point_cloud);

    // ========== 搜索方法 ==========

    /**
     * @brief k近邻搜索
     * @param point 查询点
     * @param k_nearest 要查找的最近邻点数k
     * @param Nearest_Points 输出参数：最近邻点的集合
     * @param Point_Distance 输出参数：对应点到查询点的距离
     * @param max_dist 最大搜索距离，默认为无穷大
     *
     * 查找距离查询点最近的k个点
     */
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points,
                        vector<float> & Point_Distance, double max_dist = INFINITY);

    /**
     * @brief 边界框搜索
     * @param Box_of_Point 查询的边界框
     * @param Storage 输出参数：边界框内的所有点
     *
     * 查找位于给定边界框内的所有点
     */
    void Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage);

    /**
     * @brief 半径搜索
     * @param point 查询点
     * @param radius 搜索半径
     * @param Storage 输出参数：半径范围内的所有点
     *
     * 查找距离查询点在指定半径范围内的所有点
     */
    void Radius_Search(PointType point, const float radius, PointVector &Storage);

    // ========== 插入和删除方法 ==========

    /**
     * @brief 批量添加点
     * @param PointToAdd 要添加的点的集合
     * @param downsample_on 是否启用降采样
     * @return 实际添加的点数
     *
     * 向树中批量添加点，可选择是否进行降采样
     */
    int Add_Points(PointVector & PointToAdd, bool downsample_on);

    /**
     * @brief 批量添加边界框内的点
     * @param BoxPoints 边界框的集合
     *
     * 添加位于给定边界框内的所有点
     */
    void Add_Point_Boxes(vector<BoxPointType> & BoxPoints);

    /**
     * @brief 批量删除点
     * @param PointToDel 要删除的点的集合
     *
     * 从树中批量删除指定的点
     */
    void Delete_Points(PointVector & PointToDel);

    /**
     * @brief 批量删除边界框内的点
     * @param BoxPoints 边界框的集合
     * @return 删除的点数
     *
     * 删除位于给定边界框内的所有点
     */
    int Delete_Point_Boxes(vector<BoxPointType> & BoxPoints);

    // ========== 辅助方法 ==========

    /**
     * @brief 展平树结构为点的向量
     * @param root 要展平的子树根节点
     * @param Storage 输出参数：存储展平后的点
     * @param storage_type 存储类型，控制如何处理删除的点
     *
     * 将树结构转换为线性的点集合，用于重建或导出
     */
    void flatten(KD_TREE_NODE * root, PointVector &Storage, delete_point_storage_set storage_type);

    /**
     * @brief 获取被删除的点
     * @param removed_points 输出参数：被删除的点的集合
     *
     * 获取自上次调用以来被删除的所有点
     */
    void acquire_removed_points(PointVector & removed_points);
};

#endif // IKD_TREE_H_
