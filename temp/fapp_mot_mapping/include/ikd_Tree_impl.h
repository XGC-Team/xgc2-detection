/**
 * @file ikd_Tree_impl.h
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
 * 文件说明：
 * ikd-Tree（增量式k-d树）实现文件
 * 这是一个专为机器人应用设计的动态k-d树数据结构，支持：
 * - 增量式插入和删除点云
 * - 高效的k近邻搜索
 * - 范围搜索和半径搜索
 * - 自动树重平衡
 * - 降采样功能
 * - 多线程并行重建
**/

#ifndef IKD_TREE_IMPL_H_
#define IKD_TREE_IMPL_H_

#include "ikd_Tree.h"

/* 构造函数：初始化ikd-Tree
 * 参数：
 *   delete_param：删除准则参数，当无效点比例超过此值时触发重建
 *   balance_param：平衡准则参数，当子树大小比例超过此值时触发重建
 *   box_length：降采样体素大小
 */
template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length) {
    delete_criterion_param = delete_param;      // 设置删除准则参数
    balance_criterion_param = balance_param;    // 设置平衡准则参数
    downsample_size = box_length;               // 设置降采样体素大小
    Rebuild_Logger.clear();                     // 清空重建日志队列
    termination_flag = false;                   // 初始化终止标志为false
    start_thread();                             // 启动多线程重建线程
}

/* 析构函数：清理ikd-Tree资源 */
template <typename PointType>
KD_TREE<PointType>::~KD_TREE()
{
    stop_thread();                              // 停止多线程重建线程
    Delete_Storage_Disabled = true;             // 禁用删除点存储功能
    delete_tree_nodes(&Root_Node);              // 递归删除整棵树的所有节点
    PointVector ().swap(PCL_Storage);           // 释放点云存储空间
    Rebuild_Logger.clear();                     // 清空重建日志队列
}

/* 设置删除准则参数：当无效点占比超过此阈值时触发重建 */
template <typename PointType>
void KD_TREE<PointType>::Set_delete_criterion_param(float delete_param){
    delete_criterion_param = delete_param;
}

/* 设置平衡准则参数：当左右子树大小比例失衡超过此阈值时触发重建 */
template <typename PointType>
void KD_TREE<PointType>::Set_balance_criterion_param(float balance_param){
    balance_criterion_param = balance_param;
}

/* 设置降采样参数：定义体素网格的大小 */
template <typename PointType>
void KD_TREE<PointType>::set_downsample_param(float downsample_param){
    downsample_size = downsample_param;
}

/* 初始化KD树的所有参数
 * 参数：
 *   delete_param：删除准则参数
 *   balance_param：平衡准则参数
 *   box_length：降采样体素大小
 */
template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length){
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
}

/* 初始化树节点：将节点的所有成员变量设置为默认值
 * 参数：
 *   root：待初始化的树节点指针
 */
template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE * root){
    // 初始化节点存储的点坐标为原点
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;

    // 初始化节点管辖的空间范围（包围盒）为零
    root->node_range_x[0] = 0.0f;  // x轴最小值
    root->node_range_x[1] = 0.0f;  // x轴最大值
    root->node_range_y[0] = 0.0f;  // y轴最小值
    root->node_range_y[1] = 0.0f;  // y轴最大值
    root->node_range_z[0] = 0.0f;  // z轴最小值
    root->node_range_z[1] = 0.0f;  // z轴最大值

    // 初始化分割轴（0=x, 1=y, 2=z）
    root->division_axis = 0;

    // 初始化树结构指针
    root->father_ptr = nullptr;     // 父节点指针
    root->left_son_ptr = nullptr;   // 左子节点指针
    root->right_son_ptr = nullptr;  // 右子节点指针

    // 初始化统计信息
    root->TreeSize = 0;             // 子树中的总点数
    root->invalid_point_num = 0;    // 子树中的无效点数（已删除的点）
    root->down_del_num = 0;         // 子树中因降采样而删除的点数

    // 初始化删除标志
    root->point_deleted = false;             // 节点的点是否已删除
    root->tree_deleted = false;              // 整个子树是否已删除
    root->point_downsample_deleted = false;  // 节点的点是否因降采样而删除

    // 初始化下推标志（用于延迟更新）
    root->need_push_down_to_left = false;   // 是否需要向左子树下推更新
    root->need_push_down_to_right = false;  // 是否需要向右子树下推更新

    // 初始化工作标志（用于多线程同步）
    root->working_flag = false;

    // 初始化互斥锁（用于线程安全的下推操作）
    pthread_mutex_init(&(root->push_down_mutex_lock),NULL);
}   

/* 获取树中的总点数（包括已删除的点）
 * 返回值：树中的总点数
 * 说明：如果正在重建，则使用临时缓存的大小值
 */
template <typename PointType>
int KD_TREE<PointType>::size(){
    int s = 0;
    // 如果没有正在重建，或者重建的不是根节点
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr) {
            return Root_Node->TreeSize;
        } else {
            return 0;
        }
    }
    // 如果根节点正在重建
    else {
        // 尝试获取锁
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            s = Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        } else {
            // 如果获取锁失败，返回临时缓存的大小
            return Treesize_tmp;
        }
    }
}

/* 获取整棵树的空间范围（包围盒）
 * 返回值：包含所有点的最小包围盒
 */
template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range(){
    BoxPointType range;
    // 如果没有正在重建，或者重建的不是根节点
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr) {
            // 设置包围盒的最小和最大顶点
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        } else {
            memset(&range, 0, sizeof(range));
        }
    }
    // 如果根节点正在重建
    else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex);
        } else {
            // 如果获取锁失败，返回空范围
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

/* 获取树中有效点的数量（未被删除的点）
 * 返回值：有效点数量，如果正在重建且无法获取锁则返回-1
 */
template <typename PointType>
int KD_TREE<PointType>::validnum(){
    int s = 0;
    // 如果没有正在重建，或者重建的不是根节点
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr)
            return (Root_Node->TreeSize - Root_Node->invalid_point_num);
        else
            return 0;
    }
    // 如果根节点正在重建
    else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            s = Root_Node->TreeSize-Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        } else {
            // 如果获取锁失败，返回-1表示无法获取有效信息
            return -1;
        }
    }
}

/* 获取根节点的平衡度和删除率
 * 参数：
 *   alpha_bal：输出参数，平衡度（子树大小比例）
 *   alpha_del：输出参数，删除率（无效点占比）
 */
template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del){
    // 如果没有正在重建，或者重建的不是根节点
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    }
    // 如果根节点正在重建
    else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            alpha_bal = Root_Node->alpha_bal;
            alpha_del = Root_Node->alpha_del;
            pthread_mutex_unlock(&working_flag_mutex);
            return;
        } else {
            // 如果获取锁失败，返回临时缓存的值
            alpha_bal = alpha_bal_tmp;
            alpha_del = alpha_del_tmp;
            return;
        }
    }
}

/* 启动多线程重建线程
 * 功能：初始化所有互斥锁并创建重建线程
 */
template <typename PointType>
void KD_TREE<PointType>::start_thread(){
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);          // 终止标志互斥锁
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);               // 重建指针互斥锁
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);            // 重建日志互斥锁
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL);    // 删除点缓存互斥锁
    pthread_mutex_init(&working_flag_mutex, NULL);                   // 工作标志互斥锁
    pthread_mutex_init(&search_flag_mutex, NULL);                    // 搜索标志互斥锁
    // 创建重建线程，入口函数为multi_thread_ptr
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void*) this);
    printf("Multi thread started \n");
}

/* 停止多线程重建线程
 * 功能：设置终止标志，等待线程结束，销毁所有互斥锁
 */
template <typename PointType>
void KD_TREE<PointType>::stop_thread(){
    // 设置终止标志，通知重建线程退出
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);

    // 等待重建线程结束
    if (rebuild_thread) pthread_join(rebuild_thread, NULL);

    // 销毁所有互斥锁
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);
}

/* 多线程重建的入口函数（静态函数）
 * 参数：
 *   arg：指向KD_TREE对象的指针
 * 返回值：nullptr
 * 说明：这是pthread_create所需的静态入口函数，将调用转发给成员函数
 */
template <typename PointType>
void * KD_TREE<PointType>::multi_thread_ptr(void * arg){
    KD_TREE * handle = (KD_TREE*) arg;  // 将void*转换为KD_TREE指针
    handle->multi_thread_rebuild();      // 调用实际的重建函数
    return nullptr;
}

/* 多线程重建的主循环
 * 功能：在后台线程中持续检查是否有需要重建的子树，如果有则执行重建
 * 说明：这是ikd-Tree的核心功能之一，通过异步重建保证树的平衡性，同时不阻塞主线程的操作
 */
template <typename PointType>
void KD_TREE<PointType>::multi_thread_rebuild(){
    bool terminated = false;
    KD_TREE_NODE * father_ptr;
    // KD_TREE_NODE ** new_node_ptr; // wgh: unused variable.

    // 检查是否收到终止信号
    pthread_mutex_lock(&termination_flag_mutex_lock);
    terminated = termination_flag;
    pthread_mutex_unlock(&termination_flag_mutex_lock);

    // 主循环：持续运行直到收到终止信号
    while (!terminated){
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);

        // 如果有需要重建的子树
        if (Rebuild_Ptr != nullptr ){
            /* 第一阶段：遍历并复制子树的所有点 */
            // 检查重建日志是否为空（理论上应该为空）
            if (!Rebuild_Logger.empty()){
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }

            rebuild_flag = true;  // 设置重建标志

            // 如果重建的是根节点，缓存当前的统计信息
            if (*Rebuild_Ptr == Root_Node) {
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }

            KD_TREE_NODE * old_root_node = (*Rebuild_Ptr);     // 保存旧的子树根节点
            father_ptr = (*Rebuild_Ptr)->father_ptr;           // 保存父节点指针
            PointVector ().swap(Rebuild_PCL_Storage);          // 清空重建点云存储
            // 锁定搜索操作：等待所有正在进行的搜索完成
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0){
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);  // 等待1微秒
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;  // 设置为-1表示禁止新的搜索
            pthread_mutex_unlock(&search_flag_mutex);

            // 锁定删除点缓存
            pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);

            // 展平子树，将所有有效点复制到Rebuild_PCL_Storage中
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);

            // 解锁删除点缓存
            pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);

            // 解锁搜索操作：允许新的搜索
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);

            pthread_mutex_unlock(&working_flag_mutex);   
            /* 第二阶段：重建子树并应用期间记录的操作 */
            Operation_Logger_Type Operation;
            KD_TREE_NODE * new_root_node = nullptr;

            // 如果有点需要重建
            if (int(Rebuild_PCL_Storage.size()) > 0){
                // 从点云重新构建平衡的k-d树
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size()-1, Rebuild_PCL_Storage);

                // 重建完成后，应用重建期间被阻塞的操作到新树上
                pthread_mutex_lock(&working_flag_mutex);
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                int tmp_counter = 0;

                // 处理重建日志中的所有操作
                while (!Rebuild_Logger.empty()){
                    Operation = Rebuild_Logger.front();
                    max_queue_size = max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    pthread_mutex_unlock(&working_flag_mutex);

                    // 在新树上执行该操作
                    run_operation(&new_root_node, Operation);
                    tmp_counter ++;

                    // 每处理10个操作休眠1微秒，避免长时间占用CPU
                    if (tmp_counter % 10 == 0) usleep(1);

                    pthread_mutex_lock(&working_flag_mutex);
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }  
            /* 第三阶段：将新树替换到原树的位置 */
            // pthread_mutex_lock(&working_flag_mutex);

            // 锁定搜索操作
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0){
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;  // 禁止新的搜索
            pthread_mutex_unlock(&search_flag_mutex);

            // 将新树挂到父节点上（替换旧树）
            if (father_ptr->left_son_ptr == *Rebuild_Ptr) {
                father_ptr->left_son_ptr = new_root_node;
            } else if (father_ptr->right_son_ptr == *Rebuild_Ptr){
                father_ptr->right_son_ptr = new_root_node;
            } else {
                throw "Error: Father ptr incompatible with current node\n";
            }

            // 设置新树根节点的父指针
            if (new_root_node != nullptr) new_root_node->father_ptr = father_ptr;
            (*Rebuild_Ptr) = new_root_node;
            // int valid_old = old_root_node->TreeSize-old_root_node->invalid_point_num; // wgh: unused variable.
            // int valid_new = new_root_node->TreeSize-new_root_node->invalid_point_num; // wgh: unused variable.

            // 如果父节点是静态根节点，更新逻辑根节点
            if (father_ptr == STATIC_ROOT_NODE) Root_Node = STATIC_ROOT_NODE->left_son_ptr;

            // 向上更新所有祖先节点的统计信息
            KD_TREE_NODE * update_root = *Rebuild_Ptr;
            while (update_root != nullptr && update_root != Root_Node){
                update_root = update_root->father_ptr;
                if (update_root->working_flag) break;
                if (update_root == update_root->father_ptr->left_son_ptr && update_root->father_ptr->need_push_down_to_left) break;
                if (update_root == update_root->father_ptr->right_son_ptr && update_root->father_ptr->need_push_down_to_right) break;
                Update(update_root);
            }

            // 解锁搜索操作
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);

            // 清理重建状态
            Rebuild_Ptr = nullptr;
            pthread_mutex_unlock(&working_flag_mutex);
            rebuild_flag = false;

            /* 第四阶段：删除废弃的旧树节点 */
            delete_tree_nodes(&old_root_node);
        } else {
            pthread_mutex_unlock(&working_flag_mutex);
        }
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);

        // 检查是否收到终止信号
        pthread_mutex_lock(&termination_flag_mutex_lock);
        terminated = termination_flag;
        pthread_mutex_unlock(&termination_flag_mutex_lock);

        usleep(100);  // 休眠100微秒，避免过度占用CPU
    }
    printf("Rebuild thread terminated normally\n");
}

/* 执行日志中记录的操作
 * 参数：
 *   root：操作的目标树节点
 *   operation：要执行的操作（包含操作类型和参数）
 * 说明：用于重建过程中重放被阻塞的操作
 */
template <typename PointType>
void KD_TREE<PointType>::run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation){
    switch (operation.op)
    {
    case ADD_POINT:  // 添加单个点
        Add_by_point(root, operation.point, false, (*root)->division_axis);
        break;
    case ADD_BOX:  // 添加包围盒内的点
        Add_by_range(root, operation.boxpoint, false);
        break;
    case DELETE_POINT:  // 删除单个点
        Delete_by_point(root, operation.point, false);
        break;
    case DELETE_BOX:  // 删除包围盒内的点
        Delete_by_range(root, operation.boxpoint, false, false);
        break;
    case DOWNSAMPLE_DELETE:  // 降采样删除
        Delete_by_range(root, operation.boxpoint, false, true);
        break;
    case PUSH_DOWN:  // 向下推送删除标志
        (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
        if (operation.tree_downsample_deleted) (*root)->down_del_num = (*root)->TreeSize;
        if (operation.tree_deleted) (*root)->invalid_point_num = (*root)->TreeSize;
            else (*root)->invalid_point_num = (*root)->down_del_num;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        break;
    default:
        break;
    }
}

/* 初始化构建ikd-Tree（也可用于完全重建）
 * 参数：
 *   point_cloud：用于构建树的点云
 * 说明：
 *   - 如果树已存在，先删除旧树
 *   - 使用平衡的方式构建新树
 *   - STATIC_ROOT_NODE是内存依赖上的根节点（用于方便重建）
 *   - Root_Node是逻辑上的根节点（实际使用的根节点）
 */
// wgh 初始化构建ikd-Tree；该函数也用作彻底重建Tree结构。
template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud){
    // wgh 如果已有tree结构，彻底清空之。
    if (Root_Node != nullptr){
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0) return;

    STATIC_ROOT_NODE = new KD_TREE_NODE; // wgh 内存依赖上的根节点。
    InitTreeNode(STATIC_ROOT_NODE);
    // 从点云构建平衡的k-d树，挂在STATIC_ROOT_NODE的左子节点上
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size()-1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr; // wgh 逻辑上的根节点。
}

/* k近邻搜索（关键入口函数）
 * 参数：
 *   point：查询点
 *   k_nearest：需要查找的近邻点数量
 *   Nearest_Points：输出参数，存储找到的近邻点
 *   Point_Distance：输出参数，存储对应的距离
 *   max_dist：最大搜索距离
 * 说明：使用优先队列（堆）维护当前找到的k个最近点
 */
// wgh 关键入口函数
template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(
    PointType point, int k_nearest, PointVector& Nearest_Points,
    vector<float> & Point_Distance, double max_dist)
{
    MANUAL_HEAP q(2*k_nearest); // wgh 预分配两倍空间。
    q.clear();
    // wgh 交换内存空间（实质上是交换指针地址），相当于清空了Point_Distance？
    vector<float> ().swap(Point_Distance); 
    // 如果根节点没有在重建
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        Search(Root_Node, k_nearest, point, q, max_dist); // wgh 实质的搜索函数，递归调用。
    }
    // 如果根节点正在重建，需要通过搜索互斥锁进行同步
    else {
        pthread_mutex_lock(&search_flag_mutex);
        // 等待直到允许搜索（search_mutex_counter != -1）
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;  // 增加搜索计数器
        pthread_mutex_unlock(&search_flag_mutex);

        Search(Root_Node, k_nearest, point, q, max_dist); // wgh 实质的搜索函数，递归调用。

        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;  // 减少搜索计数器
        pthread_mutex_unlock(&search_flag_mutex);
    }

    // 从堆中提取结果
    int k_found = min(k_nearest,int(q.size()));
    PointVector ().swap(Nearest_Points);
    vector<float> ().swap(Point_Distance);
    for (int i=0;i < k_found;i++){
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

/* 包围盒范围搜索
 * 参数：
 *   Box_of_Point：查询的包围盒
 *   Storage：输出参数，存储包围盒内的所有点
 */
template <typename PointType>
void KD_TREE<PointType>::Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage)
{
    Storage.clear();
    Search_by_range(Root_Node, Box_of_Point, Storage);
}

/* 半径搜索
 * 参数：
 *   point：查询点
 *   radius：搜索半径
 *   Storage：输出参数，存储半径内的所有点
 */
template <typename PointType>
void KD_TREE<PointType>::Radius_Search(PointType point, const float radius, PointVector &Storage)
{
    Storage.clear();
    Search_by_radius(Root_Node, point, radius, Storage);
}

/* 批量添加点（关键入口函数）
 * 参数：
 *   PointToAdd：待添加的点云
 *   downsample_on：是否启用降采样
 * 返回值：实际添加的点数
 * 说明：
 *   - 支持降采样功能：将点云分为体素网格，每个体素只保留最接近中心的点
 *   - 如果正在重建，需要将操作记录到日志中
 */
// wgh 关键入口函数
template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector & PointToAdd, bool downsample_on)
{
    // int NewPointSize = PointToAdd.size();   // wgh: unused variable.
    // int tree_size = size();                 // wgh: unused variable.
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    int tmp_counter = 0;  // 实际添加的点数计数器

    // wgh 遍历所有点，逐个插入。
    for (std::size_t i=0; i<PointToAdd.size();i++){
        // 如果启用降采样
        if (downsample_switch){
            // wgh 获得插入点所在的Voxel，计算Voxel的几何中心点（将来只保留最接近中心点的point）
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0]+downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1]+downsample_size; 
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2]+downsample_size;   
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0]-Box_of_Point.vertex_min[0])/2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1]-Box_of_Point.vertex_min[1])/2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2]-Box_of_Point.vertex_min[2])/2.0;
            PointVector ().swap(Downsample_Storage);
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i],mid_point);
            downsample_result = PointToAdd[i]; 
            for (std::size_t index = 0; index < Downsample_Storage.size(); index++){
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist){
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            // wgh-- 如果当前没有re-balancing任务，也即没有并行线程，则直接执行`BoxDelete`和`插入一个点`。
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){  
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)){
                    if (Downsample_Storage.size() > 0) Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
                    tmp_counter ++;                      
                }
            // wgh-- 如果有re-balancing任务在并行运行，在对当前tree执行`BoxDelete`和`插入一个点`之外，还需要把这些操作缓存到logger里。
            } else {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)){
                    Operation_Logger_Type  operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    pthread_mutex_lock(&working_flag_mutex);
                    if (Downsample_Storage.size() > 0) Delete_by_range(&Root_Node, Box_of_Point, false , true);                                      
                    Add_by_point(&Root_Node, downsample_result, false, Root_Node->division_axis);
                    tmp_counter ++;
                    if (rebuild_flag){
                        pthread_mutex_lock(&rebuild_logger_mutex_lock);
                        if (Downsample_Storage.size() > 0) Rebuild_Logger.push(operation_delete);
                        Rebuild_Logger.push(operation);
                        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    }
                    pthread_mutex_unlock(&working_flag_mutex);
                };
            }
        }
        else {
            // wgh 如果不需要降采样，且无并行re-balancing任务，直接插入点。
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
                Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node->division_axis);     
            } 
            // wgh 如果有并行re-balancing任务，还需额外把当前操作放入logger缓存。
            else {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;                
                pthread_mutex_lock(&working_flag_mutex);
                Add_by_point(&Root_Node, PointToAdd[i], false, Root_Node->division_axis);
                if (rebuild_flag){
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    Rebuild_Logger.push(operation);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&working_flag_mutex);       
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> & BoxPoints){     
    for (std::size_t i=0;i < BoxPoints.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
            Add_by_range(&Root_Node ,BoxPoints[i], true);
        } else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_range(&Root_Node ,BoxPoints[i], false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }               
            pthread_mutex_unlock(&working_flag_mutex);
        }    
    } 
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector & PointToDel){        
    for (std::size_t i=0;i<PointToDel.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){               
            Delete_by_point(&Root_Node, PointToDel[i], true);
        } else {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            pthread_mutex_lock(&working_flag_mutex);        
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }      
    }      
    return;
}

/* 批量删除包围盒内的点（关键入口函数）
 * 参数：
 *   BoxPoints：待删除的包围盒数组
 * 返回值：实际删除的点数
 * 说明：遍历所有包围盒，删除每个包围盒内的所有点
 */
// wgh 关键入口函数，删除指定Box内的点
template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> & BoxPoints){
    int tmp_counter = 0;  // 删除点数计数器
    // wgh 遍历所有box，逐个删除
    for (std::size_t i=0;i < BoxPoints.size();i++){
        // wgh 无并行线程时，直接执行删除
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
            tmp_counter += Delete_by_range(&Root_Node ,BoxPoints[i], true, false);
        }
        // wgh 如果此时有并行的re-balancing线程，需要通过锁(互斥量)访问
        else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;     
            pthread_mutex_lock(&working_flag_mutex); 
            tmp_counter += Delete_by_range(&Root_Node ,BoxPoints[i], false, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }                
            pthread_mutex_unlock(&working_flag_mutex);
        }
    } 
    return tmp_counter;
}

/* 获取已删除的点
 * 参数：
 *   removed_points：输出参数，存储所有已删除的点
 * 说明：
 *   - 从两个缓存中获取删除的点：Points_deleted（单线程删除）和Multithread_Points_deleted（多线程重建时删除）
 *   - 获取后清空缓存
 */
template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector & removed_points){
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
    // 收集单线程删除的点
    for (std::size_t i = 0; i < Points_deleted.size();i++){
        removed_points.push_back(Points_deleted[i]);
    }
    // 收集多线程重建时删除的点
    for (std::size_t i = 0; i < Multithread_Points_deleted.size();i++){
        removed_points.push_back(Multithread_Points_deleted[i]);
    }
    // 清空缓存
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
    return;
}

/* 构建平衡k-d树（递归函数）
 * 参数：
 *   root：双指针，用于返回构建的树根节点（允许传入空指针）
 *   l：点云数组的左边界索引
 *   r：点云数组的右边界索引
 *   Storage：存储点云的数组
 * 说明：
 *   - 选择分布最分散的轴作为分割轴
 *   - 使用中位数分割，保证树的平衡性
 *   - 递归构建左右子树
 */
// wgh 工具属性，构建新的(sub)tree；这里的双指针形参很重要，能够允许我们传入一个空指针。
template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage){
    if (l>r) return;  // 递归终止条件

    *root = new KD_TREE_NODE;
    InitTreeNode(*root);
    int mid = (l+r)>>1;  // 中位数索引
    int div_axis = 0;     // 分割轴
    int i;

    // Find the best division Axis (wgh 也即分布最分散的那个轴，或者说最大值减最小值之差最大的那个轴)
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0,0,0};
    // 遍历所有点，找到每个维度的最小值和最大值
    for (i=l;i<=r;i++){
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }

    // Select the longest dimension as division axis
    // 计算每个维度的范围
    for (i=0;i<3;i++) dim_range[i] = max_value[i] - min_value[i];
    // 选择范围最大的维度作为分割轴
    for (i=1;i<3;i++) if (dim_range[i] > dim_range[div_axis]) div_axis = i;

    // Divide by the division axis and recursively build.
    (*root)->division_axis = div_axis;

    // wgh 以下，应该是按照主轴方向排序，排序结果放在Storage变量中。
    // 根据分割轴对点进行部分排序
    switch (div_axis)
    {
    case 0:
        // wgh 用C++算法库的函数进行排序，只需确保在mid位置的数大于左侧，且小于右侧即可，不必严格完全排序。
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_x);
        break;
    case 1:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_y);
        break;
    case 2:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_x);
        break;
    }

    // 中位数点作为当前节点的点
    (*root)->point = Storage[mid];
    KD_TREE_NODE * left_son = nullptr, * right_son = nullptr;

    // wgh 递归构建整个tree（自上而下）。
    BuildTree(&left_son, l, mid-1, Storage);      // 递归构建左子树
    BuildTree(&right_son, mid+1, r, Storage);     // 递归构建右子树
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;

    // wgh 更新根节点信息。
    Update((*root));
    return;
}

/* 重建子树
 * 参数：
 *   root：需要重建的子树根节点
 * 说明：
 *   - 如果子树足够大（>= Multi_Thread_Rebuild_Point_Num），使用多线程异步重建
 *   - 如果子树较小，直接在当前线程同步重建
 */
template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE ** root){
    KD_TREE_NODE * father_ptr;
    // 如果子树足够大，使用多线程异步重建
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num) {
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock)){
            // 如果没有正在重建的树，或者当前树更大，则设置为重建目标
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) {
                Rebuild_Ptr = root;
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    }
    // 如果子树较小，直接同步重建
    else {
        father_ptr = (*root)->father_ptr;
        // int size_rec = (*root)->TreeSize; // wgh: unused variable.
        PCL_Storage.clear();
        // 展平子树，获取所有有效点
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        // 删除旧树
        delete_tree_nodes(root);
        // 从点云重新构建平衡树
        BuildTree(root, 0, PCL_Storage.size()-1, PCL_Storage);
        // 恢复父指针
        if (*root != nullptr) (*root)->father_ptr = father_ptr;
        if (*root == Root_Node) STATIC_ROOT_NODE->left_son_ptr = *root;
    }
    return;
}

/* 按范围删除点（递归函数）
 * 参数：
 *   root：当前搜索的子树根节点
 *   boxpoint：删除范围（包围盒）
 *   allow_rebuild：是否允许触发重建
 *   is_downsample：是否为降采样删除
 * 返回值：删除的点数
 * 说明：
 *   - 仅标记删除，不实际删除节点
 *   - 如果包围盒完全包含节点的空间范围，则标记整个子树删除
 *   - 递归搜索左右子树
 *   - 删除后检查是否需要重建
 */
// wgh 工具属性，从根节点开始向下递归搜索，删除（仅标记）所有被Box包含的节点。
template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE ** root,  BoxPointType boxpoint,
                                        bool allow_rebuild, bool is_downsample)
{
    if ((*root) == nullptr || (*root)->tree_deleted) return 0;
    (*root)->working_flag = true;
    Push_Down(*root);// wgh 向下更新信息。

    int tmp_counter = 0;  // 删除点数计数器

    // wgh 当且仅当两个空间有交叉时，才有继续的必要。
    // 如果包围盒与节点空间没有交集，直接返回
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return 0;

    // wgh 当Box完全包含了节点所张成的空间时，把该节点subtree上的所有点标记删除。
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && 
        boxpoint.vertex_max[0] > (*root)->node_range_x[1] && 
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && 
        boxpoint.vertex_max[1] > (*root)->node_range_y[1] && 
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && 
        boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample){
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }
        return tmp_counter;
    }

    // wgh 如果当前节点的point被Box包含，标记删除该point。
    if (!(*root)->point_deleted && 
        boxpoint.vertex_min[0] <= (*root)->point.x && 
        boxpoint.vertex_max[0] > (*root)->point.x && 
        boxpoint.vertex_min[1] <= (*root)->point.y && 
        boxpoint.vertex_max[1] > (*root)->point.y && 
        boxpoint.vertex_min[2] <= (*root)->point.z && 
        boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample) (*root)->point_downsample_deleted = true;
    }

    //
    Operation_Logger_Type delete_box_log;
    // struct timespec Timeout; // wgh: unused variable.
    if (is_downsample) delete_box_log.op = DOWNSAMPLE_DELETE;
        else delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;

    // 左子树递归删除
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } 
    else {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }

    // 右子树递归删除
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } 
    else {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }    

    Update(*root); // wgh 更新当前节点信息（自下而上更新）
    if (Rebuild_Ptr != nullptr && 
        *Rebuild_Ptr == *root && 
        (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    
    // wgh 检查是否需要re-balancing当前子树。
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if ((*root) != nullptr) (*root)->working_flag = false;
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild){   
    if ((*root) == nullptr || (*root)->tree_deleted) return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted) {          
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize) (*root)->tree_deleted = true;    
        return;
    }
    Operation_Logger_Type delete_log;
    // struct timespec Timeout; // wgh: unused variable.
    delete_log.op = DELETE_POINT;
    delete_log.point = point;     
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){           
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);         
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->left_son_ptr, point,false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    } else {       
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);         
        } else {
            pthread_mutex_lock(&working_flag_mutex); 
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }        
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if ((*root) != nullptr) (*root)->working_flag = false;   
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild){
    if ((*root) == nullptr) return;
    (*root)->working_flag = true;
    Push_Down(*root);       
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1]> (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]){
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num; 
        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z){
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Operation_Logger_Type add_box_log;
    // struct timespec Timeout; // wgh: unused variable.
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }        
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if ((*root) != nullptr) (*root)->working_flag = false;   
    return;
}

/* 按点插入（递归函数）
 * 参数：
 *   root：当前搜索的子树根节点
 *   point：待插入的点
 *   allow_rebuild：是否允许触发重建
 *   father_axis：父节点的分割轴（用于确定当前节点的分割轴）
 * 说明：
 *   - 根据分割轴判断插入左子树还是右子树
 *   - 如果到达叶子节点（nullptr），创建新节点
 *   - 插入后向上更新节点信息，并检查是否需要重建
 */
// wgh 工具属性，单纯地往tree结构中插入新节点。
template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild, int father_axis)
{
    // wgh 如果已经到达叶子节点，直接插入。
    if (*root == nullptr){
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;  // 分割轴循环切换
        Update(*root);
        return;
    }

    // wgh `工作中`标志位置true，同步记录到Logger中。
    (*root)->working_flag = true;
    Operation_Logger_Type add_log;
    // struct timespec Timeout; // wgh: unused variable.
    add_log.op = ADD_POINT;
    add_log.point = point;
    Push_Down(*root);  // 向下推送待处理的删除标志

    // wgh 递归插入左子树。
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || 
        ((*root)->division_axis == 1 && point.y < (*root)->point.y) || 
        ((*root)->division_axis == 2 && point.z < (*root)->point.z) )
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){ 
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
        } 
        else {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->left_son_ptr, point, false,(*root)->division_axis);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);            
        }
    }
    // wgh 递归插入右子树。
    else {  
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild,(*root)->division_axis);
        } 
        else {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->right_son_ptr, point, false,(*root)->division_axis);       
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex); 
        }
    }

    // wgh 更新当前节点信息（自下而上），并检查是否需要re-balancing。
    Update(*root);   
    if (Rebuild_Ptr != nullptr && 
        *Rebuild_Ptr == *root && 
        (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root); 
    if ((*root) != nullptr) (*root)->working_flag = false;   
    return;
}

/* k近邻搜索的递归实现
 * 参数：
 *   root：当前搜索的子树根节点
 *   k_nearest：需要查找的近邻点数量
 *   point：查询点
 *   q：优先队列（堆），维护当前找到的k个最近点
 *   max_dist：最大搜索距离
 * 说明：
 *   - 使用包围盒剪枝：如果节点的包围盒距离查询点太远，跳过该子树
 *   - 使用bounds-overlap-ball策略：优先搜索距离更近的子树
 *   - 如果堆未满或当前点比堆顶更近，则加入堆中
 */
template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist)
{
    // wgh 如果整个subtree被标记为treedeleted，直接退出。
    if (root == nullptr || root->tree_deleted) return;

    // wgh 如同论文中讲到的，搜索到任一个节点时，首先根据节点的range信息，比较节点张成的空间是否与point距离张成的球空间有交叉？
    // wgh 如果无交叉，则不可能存在ranged-kNN解，直接退出（剪枝加速）。
    double cur_dist = calc_box_dist(root, point);
    double max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr) return;

    // wgh 如果当前节点需要更新状态信息，则先更新。
    int retval;
    if (root->need_push_down_to_left || root->need_push_down_to_right) {
        // 尝试获取下推锁
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0){
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        } else {
            // 如果获取锁失败，等待锁释放
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }

    // wgh 只要当前节点未被标记为删除，则计算当前节点到point的距离，如果在range之内，就放入结果缓存队列。
    if (!root->point_deleted){
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist)){
            if (q.size() >= k_nearest) q.pop();
            PointType_CMP current_point{root->point, dist};                    
            q.push(current_point);            
        }
    }  

    // wgh 继续向下递归搜索。「逻辑核心*」
    // int cur_search_counter; // wgh: unused variable.
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    // wgh 如果NN数量不足k个，且左枝或右枝可能存在NN。
    if (q.size()< k_nearest || (dist_left_node < q.top().dist && dist_right_node < q.top().dist)){
        // wgh 优先搜索距离更小的分支
        if (dist_left_node <= dist_right_node) {
            // wgh 如果无并行任务，直接递归搜索
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);                       
            } 
            // wgh 如果有并行任务，仅在允许`读`时再执行递归搜索。
            else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            // wgh bounds-overlap-ball搜索法（实现策略略有不同，效果一样）
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);                    
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);  
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }                
            }
        } else {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);                   
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {            
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);  
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);  
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    } 
    // wgh 如果NN数量已有k个，当且仅当且左枝或右枝可能存在优于`当前最差解`的情况下，继续进行递归搜索。
    else {
        if (dist_left_node < q.top().dist) {        
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);  
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);  
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }
    return;
}

/* 按范围搜索点（递归函数）
 * 参数：
 *   root：当前搜索的子树根节点
 *   boxpoint：查询的包围盒
 *   Storage：输出参数，存储包围盒内的所有点
 * 说明：
 *   - 如果包围盒与节点空间没有交集，剪枝
 *   - 如果包围盒完全包含节点空间，展平整个子树
 *   - 否则递归搜索左右子树
 */
// wgh 工具属性，从根节点开始向下递归搜索，获得所有被Box包含的节点。
template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector & Storage){
    if (root == nullptr) return;
    Push_Down(root); // wgh 向下更新信息。

    // wgh 当且仅当两个空间有交叉时，才有继续的必要。
    // 如果包围盒与节点空间没有交集，剪枝
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1]) return;

    // wgh 当Box完全包含了节点所张成的空间时，把该节点subtree上的所有点返回。
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && 
        boxpoint.vertex_max[0] > root->node_range_x[1] && 
        boxpoint.vertex_min[1] <= root->node_range_y[0] && 
        boxpoint.vertex_max[1] > root->node_range_y[1] && 
        boxpoint.vertex_min[2] <= root->node_range_z[0] && 
        boxpoint.vertex_max[2] > root->node_range_z[1])
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }

    // wgh 如果当前节点的point被Box包含，记录该point。
    if (boxpoint.vertex_min[0] <= root->point.x && 
        boxpoint.vertex_max[0] > root->point.x && 
        boxpoint.vertex_min[1] <= root->point.y && 
        boxpoint.vertex_max[1] > root->point.y && 
        boxpoint.vertex_min[2] <= root->point.z && 
        boxpoint.vertex_max[2] > root->point.z)
    {
        if (!root->point_deleted) Storage.push_back(root->point);
    }

    // wgh 递归搜索左子树。
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    } 
    else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    // wgh 递归搜索右子树。
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    } 
    else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }

    return;
}

/* 按半径搜索点（递归函数）
 * 参数：
 *   root：当前搜索的子树根节点
 *   point：查询点（球心）
 *   radius：搜索半径
 *   Storage：输出参数，存储半径内的所有点
 * 说明：
 *   - 计算节点包围盒中心到查询点的距离
 *   - 如果球与包围盒没有交集，剪枝
 *   - 如果包围盒完全在球内，展平整个子树
 *   - 否则递归搜索左右子树
 */
template <typename PointType>
void KD_TREE<PointType>::Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);

    // 计算节点包围盒的中心点
    PointType range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;

    // 计算包围盒中心到查询点的距离
    float dist = sqrt(calc_dist(range_center, point));

    // 如果球与包围盒没有交集，剪枝
    if (dist > radius + sqrt(root->radius_sq)) return;

    // 如果包围盒完全在球内，展平整个子树
    if (dist <= radius - sqrt(root->radius_sq))
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }

    // 检查当前节点的点是否在半径内
    if (!root->point_deleted && calc_dist(root->point, point) <= radius * radius){
        Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }    
    return;
}

/* 检查是否需要重建
 * 参数：
 *   root：待检查的节点
 * 返回值：true表示需要重建，false表示不需要
 * 说明：
 *   - 如果树太小，不需要重建
 *   - 如果删除率过高，需要重建
 *   - 如果平衡度过差（左右子树大小差异过大），需要重建
 */
template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE * root){
    // 如果树太小，不需要重建
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size){
        return false;
    }

    float balance_evaluation = 0.0f;  // 平衡度评估
    float delete_evaluation = 0.0f;   // 删除率评估
    KD_TREE_NODE * son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr) son_ptr = root->right_son_ptr;

    // 计算删除率：无效点数 / 总点数
    delete_evaluation = float(root->invalid_point_num)/ root->TreeSize;
    // 计算平衡度：子树大小 / (总大小-1)
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize-1);

    // 如果删除率过高，需要重建
    if (delete_evaluation > delete_criterion_param){
        return true;
    }
    // 如果平衡度过差，需要重建
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1-balance_criterion_param){
        return true;
    }
    return false;
}

/* 向下推送删除标志（延迟更新）
 * 参数：
 *   root：当前节点
 * 说明：
 *   - 如果节点被标记为删除，需要将删除标志下推到子节点
 *   - 这是一种延迟更新策略，避免每次删除都遍历整个子树
 *   - 在访问节点时才实际执行下推操作
 */
template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root){
    if (root == nullptr) return;

    // 构造下推操作日志
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;

    // 如果需要向左子节点下推且左子节点存在
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
                else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;                
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
                else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;            
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_left = false;
            pthread_mutex_unlock(&working_flag_mutex);            
        }
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
                else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
                else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;            
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }            
            root->need_push_down_to_right = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

/* 更新节点的统计信息（从子节点向上更新）
 * 参数：
 *   root：待更新的节点
 * 说明：
 *   - 更新节点的包围盒范围（node_range_x/y/z）
 *   - 更新树大小（TreeSize）
 *   - 更新无效点数（invalid_point_num）
 *   - 更新降采样删除点数（down_del_num）
 *   - 更新删除标志（tree_deleted, point_deleted）
 *   - 更新半径平方（radius_sq）
 *   - 如果是根节点，更新平衡度和删除率（alpha_bal, alpha_del）
 */
// wgh 工具属性，更新指定节点的min/max-range, treesize, radius_sq, alpha_del等信息。
template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE * root){
    KD_TREE_NODE * left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE * right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};

    // Update Tree Size（更新树大小）   
    if (left_son_ptr != nullptr && right_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted? 1:0);
        root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted? 1:0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || 
            (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted && !root->point_deleted) ){
            tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0],right_son_ptr->node_range_x[0]),root->point.x);
            tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1],right_son_ptr->node_range_x[1]),root->point.x);
            tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0],right_son_ptr->node_range_y[0]),root->point.y);
            tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1],right_son_ptr->node_range_y[1]),root->point.y);
            tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0],right_son_ptr->node_range_z[0]),root->point.z);
            tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1],right_son_ptr->node_range_z[1]),root->point.z);
        } 
        else {
            if (!left_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);                
            }
            if (!root->point_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);                 
            }
        }
    } 
    else if (left_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted?1:0);
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted?1:0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted)){
            tmp_range_x[0] = min(left_son_ptr->node_range_x[0],root->point.x);
            tmp_range_x[1] = max(left_son_ptr->node_range_x[1],root->point.x);
            tmp_range_y[0] = min(left_son_ptr->node_range_y[0],root->point.y);
            tmp_range_y[1] = max(left_son_ptr->node_range_y[1],root->point.y); 
            tmp_range_z[0] = min(left_son_ptr->node_range_z[0],root->point.z);
            tmp_range_z[1] = max(left_son_ptr->node_range_z[1],root->point.z);  
        } else {
            if (!left_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);                
            }
            if (!root->point_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);                 
            }            
        }

    } 
    else if (right_son_ptr != nullptr){
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted? 1:0);
        root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted? 1:0);        
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted)){
            tmp_range_x[0] = min(right_son_ptr->node_range_x[0],root->point.x);
            tmp_range_x[1] = max(right_son_ptr->node_range_x[1],root->point.x);
            tmp_range_y[0] = min(right_son_ptr->node_range_y[0],root->point.y);
            tmp_range_y[1] = max(right_son_ptr->node_range_y[1],root->point.y); 
            tmp_range_z[0] = min(right_son_ptr->node_range_z[0],root->point.z);
            tmp_range_z[1] = max(right_son_ptr->node_range_z[1],root->point.z); 
        } else {
            if (!right_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);                
            }
            if (!root->point_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);                 
            }            
        }
    } else {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted? 1:0);
        root->down_del_num = (root->point_downsample_deleted? 1:0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;        
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y; 
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;                 
    }
    // wgh 用memcpy函数直接拷贝内存空间，节约时间花销。
    memcpy(root->node_range_x,tmp_range_x,sizeof(tmp_range_x));
    memcpy(root->node_range_y,tmp_range_y,sizeof(tmp_range_y));
    memcpy(root->node_range_z,tmp_range_z,sizeof(tmp_range_z));
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    root->radius_sq = x_L*x_L + y_L * y_L + z_L * z_L;    
    if (left_son_ptr != nullptr) left_son_ptr -> father_ptr = root;
    if (right_son_ptr != nullptr) right_son_ptr -> father_ptr = root;
    if (root == Root_Node && root->TreeSize > 3){
        KD_TREE_NODE * son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize-1);
        root->alpha_del = float(root->invalid_point_num)/ root->TreeSize;
        root->alpha_bal = (tmp_bal>=0.5-EPSS)?tmp_bal:1-tmp_bal;
    }   
    return;
}

/* 展平子树（将子树的所有有效点收集到数组中）
 * 参数：
 *   root：待展平的子树根节点
 *   Storage：输出参数，存储所有有效点
 *   storage_type：存储类型，决定如何处理已删除的点
 *     - NOT_RECORD：不记录删除的点
 *     - DELETE_POINTS_REC：将删除的点记录到Points_deleted中
 *     - MULTI_THREAD_REC：将删除的点记录到Multithread_Points_deleted中
 * 说明：
 *   - 递归遍历整个子树
 *   - 收集所有未被删除的点
 *   - 根据storage_type记录已删除的点
 */
template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE * root, PointVector &Storage, delete_point_storage_set storage_type){
    if (root == nullptr) return;
    Push_Down(root);  // 下推删除标志

    // 如果点未被删除，收集该点
    if (!root->point_deleted) {
        Storage.push_back(root->point);
    }

    // 递归展平左右子树
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);

    // 根据存储类型处理已删除的点
    switch (storage_type)
    {
    case NOT_RECORD:  // 不记录删除的点
        break;
    case DELETE_POINTS_REC:  // 单线程删除，记录到Points_deleted
        if (root->point_deleted && !root->point_downsample_deleted) {
            Points_deleted.push_back(root->point);
        }
        break;
    case MULTI_THREAD_REC:  // 多线程重建，记录到Multithread_Points_deleted
        if (root->point_deleted  && !root->point_downsample_deleted) {
            Multithread_Points_deleted.push_back(root->point);
        }
        break;
    default:
        break;
    }
    return;
}

/* 递归删除整个子树并释放内存
 * 参数：
 *   root：待删除的子树根节点（双指针）
 * 说明：
 *   - 后序遍历删除所有节点
 *   - 销毁每个节点的互斥锁
 *   - 释放节点内存
 */
// wgh 工具属性，递归释放整个(sub)tree的内存空间。
template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE ** root){
    if (*root == nullptr) return;
    Push_Down(*root);    // 先下推删除标志

    // 递归删除左右子树
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);

    // 销毁节点的互斥锁
    pthread_mutex_destroy( &(*root)->push_down_mutex_lock);
    // 释放节点内存
    delete *root;
    *root = nullptr;

    return;
}

/* 判断两个点是否相同
 * 参数：
 *   a, b：待比较的两个点
 * 返回值：如果两点的x、y、z坐标差值都小于EPSS，返回true
 * 说明：使用浮点数容差比较，避免浮点数精度问题
 */
template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b){
    return (fabs(a.x-b.x) < EPSS && fabs(a.y-b.y) < EPSS && fabs(a.z-b.z) < EPSS );
}

/* 计算两点之间的欧氏距离的平方
 * 参数：
 *   a, b：待计算距离的两个点
 * 返回值：距离的平方（避免开方运算，提高效率）
 */
template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b){
    float dist = 0.0f;
    dist = (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z);
    return dist;
}

/* 计算点到节点包围盒的最小距离的平方
 * 参数：
 *   node：树节点
 *   point：查询点
 * 返回值：点到包围盒的最小距离的平方
 * 说明：
 *   - 如果点在包围盒内，返回0
 *   - 如果点在包围盒外，返回点到包围盒最近点的距离的平方
 *   - 用于搜索时的剪枝判断
 */
template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE * node, PointType point){
    if (node == nullptr) return INFINITY;
    float min_dist = 0.0;
    // wgh 因为算出来的distance仅用于比较，因此只需算出distance的平方就行，无需开方。
    // 计算x方向的距离
    if (point.x < node->node_range_x[0]) min_dist += (point.x - node->node_range_x[0])*(point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1]) min_dist += (point.x - node->node_range_x[1])*(point.x - node->node_range_x[1]);
    // 计算y方向的距离
    if (point.y < node->node_range_y[0]) min_dist += (point.y - node->node_range_y[0])*(point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1]) min_dist += (point.y - node->node_range_y[1])*(point.y - node->node_range_y[1]);
    // 计算z方向的距离
    if (point.z < node->node_range_z[0]) min_dist += (point.z - node->node_range_z[0])*(point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1]) min_dist += (point.z - node->node_range_z[1])*(point.z - node->node_range_z[1]);
    return min_dist;
}

/* 点比较函数：按x坐标排序 */
template <typename PointType> bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x;}
/* 点比较函数：按y坐标排序 */
template <typename PointType> bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y;}
/* 点比较函数：按z坐标排序 */
template <typename PointType> bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z;}

/* ========================================
 * 手动队列（MANUAL_Q）实现
 * 说明：
 *   - 这是一个环形队列（循环队列）实现
 *   - 用于缓存操作日志
 *   - 避免使用STL队列的动态内存分配开销
 * ======================================== */

/* 清空队列 */
template <typename T>
void MANUAL_Q<T>::clear(){
    head = 0;
    tail = 0;
    counter = 0;
    is_empty = true;
    return;
}

/* 弹出队首元素 */
template <typename T>
void MANUAL_Q<T>::pop(){
    if (counter == 0) return;
    head ++;
    head %= Q_LEN;  // 循环队列：索引回绕
    counter --;
    if (counter == 0) is_empty = true;
    return;
}

/* 获取队首元素 */
template <typename T>
T MANUAL_Q<T>::front(){
    return q[head];
}

/* 获取队尾元素 */
template <typename T>
T MANUAL_Q<T>::back(){
    return q[tail];
}

/* 在队尾插入元素 */
template <typename T>
void MANUAL_Q<T>::push(T op){
    q[tail] = op;
    counter ++;
    if (is_empty) is_empty = false;
    tail ++;
    tail %= Q_LEN;  // 循环队列：索引回绕
}

/* 判断队列是否为空 */
template <typename T>
bool MANUAL_Q<T>::empty(){
    return is_empty;
}

/* 获取队列中的元素数量 */
template <typename T>
int MANUAL_Q<T>::size(){
    return counter;
}


#endif // IKD_TREE_IMPL_H_

// template class KD_TREE<ikdTree_PointType>;
// template class KD_TREE<pcl::PointXYZ>;
// template class KD_TREE<pcl::PointXYZI>;
// template class KD_TREE<pcl::PointXYZINormal>;