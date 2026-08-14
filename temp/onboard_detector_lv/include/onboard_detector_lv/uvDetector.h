/*
    FILE: uvDetector.h
    ------------------
    UV检测器辅助类头文件

    功能说明：
    该文件定义了基于UV映射的障碍物检测系统，主要包括：
    - UVbox: UV空间中的边界框表示
    - UVtracker: 基于鸟瞰图的目标跟踪器
    - UVdetector: 完整的UV检测器，包含深度图处理、U图提取、边界框检测等功能

    核心思想：
    通过将深度图投影到UV空间（鸟瞰图），简化障碍物检测问题，
    实现快速、鲁棒的3D目标检测和跟踪
*/
#ifndef UV_DETECTOR_H
#define UV_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <math.h>
#include <vector>
#include <onboard_detector_lv/utils.h>
#include <onboard_detector_lv/kalmanFilter.h>
#include <queue>
#include <Eigen/Dense>

namespace onboardDetector{

    /**
     * @brief UV空间边界框类
     *
     * 用于在UV映射空间中表示检测到的目标边界框。
     * 支持边界框的层次结构管理，用于合并和追踪相邻的检测区域。
     */
    class UVbox
    {
        public:
        // 成员变量
        int id; // 当前边界框的唯一标识符
        int toppest_parent_id; // 最顶层父边界框的ID，用于追踪合并关系
        cv::Rect bb; // OpenCV矩形对象，存储边界框的位置和尺寸 (x, y, width, height)

        /**
         * @brief 默认构造函数
         *
         * 创建一个空的UV边界框对象
         */
        UVbox();

        /**
         * @brief 从扫描线信息构造边界框
         *
         * @param seg_id 分割ID，用于标识该边界框所属的分割区域
         * @param row 扫描线所在的行号
         * @param left 边界框左边界的列坐标
         * @param right 边界框右边界的列坐标
         */
        UVbox(int seg_id, int row, int left, int right);
    };



    /**
     * @brief UV空间目标跟踪器类
     *
     * 在鸟瞰图（Bird's Eye View）空间中对检测到的目标进行跟踪。
     * 使用卡尔曼滤波器预测目标运动状态，并维护目标的历史轨迹信息。
     * 支持2D边界框到3D边界框的关联和速度估计。
     */
    class UVtracker
    {
        public:
        // 边界框信息
        std::vector<cv::Rect> pre_bb; // 上一帧的2D边界框列表（鸟瞰图空间）
        std::vector<cv::Rect> now_bb; // 当前帧的2D边界框列表（鸟瞰图空间）

        // 历史轨迹信息
        std::vector<vector<cv::Point2f> > pre_history; // 上一帧各目标的历史检测中心点序列
        std::vector<vector<cv::Point2f> > now_history; // 当前帧各目标的历史检测中心点序列

        // 卡尔曼滤波器状态估计
        std::vector<kalman_filter> pre_filter; // 上一帧的卡尔曼滤波器列表，状态包括：x, y, vx, vy, width, depth
        std::vector<kalman_filter> now_filter; // 当前帧的卡尔曼滤波器列表

        // 深度图和3D边界框信息
        std::vector<cv::Rect> now_bb_D; // 当前帧在深度图上的边界框列表
        std::vector<box3D> now_box_3D; // 当前帧的3D边界框列表
        std::deque<deque<box3D>> now_box_3D_history; // 当前帧各目标的3D边界框历史队列
        std::deque<deque<box3D>> pre_box_3D_history; // 上一帧各目标的3D边界框历史队列

        float overlap_threshold; // 边界框重叠阈值，用于判断目标是否成功跟踪（IoU阈值）

        // 速度预测和统计
        std::deque<std::deque<Eigen::MatrixXd>> pre_V; // 上一帧各目标的预测速度累加和，用于计算平均速度
        std::deque<std::deque<Eigen::MatrixXd>> now_V; // 当前帧各目标的预测速度累加和

        // 运动状态统计
        std::deque<std::deque<int>> pre_count; // 上一帧各目标被识别为运动状态的次数统计
        std::deque<std::deque<int>> now_count; // 当前帧各目标被识别为运动状态的次数统计

        // 固定尺寸信息
        std::vector<box3D> fixed_box3D; // 存储目标在视野中完全可见时的固定3D边界框尺寸

        // 尺寸固定标志（已注释）
        // vector<bool> pre_fix; // 上一帧各目标尺寸是否已固定的标志
        // vector<bool> now_fix; // 当前帧各目标尺寸是否已固定的标志

        /**
         * @brief 构造函数
         *
         * 初始化UV跟踪器，设置默认参数（如重叠阈值等）
         */
        UVtracker();

        /**
         * @brief 读取新的边界框信息
         *
         * 将检测到的2D和3D边界框信息更新到跟踪器中，
         * 准备进行下一步的跟踪和关联操作。
         *
         * @param now_bb 当前帧在鸟瞰图上的2D边界框列表
         * @param now_bb_D 当前帧在深度图上的2D边界框列表
         * @param box_3D 当前帧的3D边界框列表（输入输出参数，可能被修改）
         */
        void read_bb(vector<cv::Rect> now_bb, vector<cv::Rect> now_bb_D, vector<box3D> &box_3D);

        /**
         * @brief 检查跟踪状态
         *
         * 将当前检测结果与历史跟踪结果进行关联，
         * 更新每个目标的跟踪状态、ID和运动参数。
         * 使用卡尔曼滤波器进行状态预测和更新。
         *
         * @param box_3D 当前帧的3D边界框列表（输入输出参数，会被更新跟踪ID等信息）
         */
        void check_status(vector<box3D> &box_3D);


    };

    /**
     * @brief UV检测器主类
     *
     * 完整的基于UV映射的3D目标检测系统，整合了深度图处理、
     * U图提取、边界框检测、鸟瞰图生成和目标跟踪等功能。
     *
     * 工作流程：
     * 1. 读取深度图和RGB图像
     * 2. 提取U图（深度不连续性图）
     * 3. 在U图上检测边界框
     * 4. 生成鸟瞰图并提取3D边界框
     * 5. 使用跟踪器关联和跟踪目标
     * 6. 输出带有ID和速度信息的3D边界框
     */
    class UVdetector
    {
        public:
        // 图像数据成员
        cv::Mat depth; // 原始深度图
        cv::Mat depth_show; // 用于显示的深度图（可视化）
        // Mat depth1; // 备用深度图1（已注释）
        // Mat depth2; // 备用深度图2（已注释）

        cv::Mat RGB; // RGB彩色图像
        cv::Mat depth_low_res; // 降采样后的低分辨率深度图
        cv::Mat U_map; // U图，表示深度不连续性的二值图或梯度图
        cv::Mat U_map_show; // 用于显示的U图（可视化）

        // 检测参数
        int min_dist; // 感兴趣深度范围的下界（最小距离，单位：毫米或米）
        int max_dist; // 感兴趣深度范围的上界（最大距离，单位：毫米或米）
        int row_downsample; // 行降采样比例（深度图高度 / U图高度）
        float col_scale; // 列方向缩放因子，用于水平方向的图像缩放
        float threshold_point; // 感兴趣点的阈值，用于判断深度不连续性
        float threshold_line; // 感兴趣线段的阈值，用于连接检测点
        int min_length_line; // 线段的最小长度，过滤掉过短的检测

        bool show_bounding_box_U; // 是否显示U图上的边界框标志

        // 检测结果
        std::vector<cv::Rect> bounding_box_U; // U图上提取的2D边界框列表
        std::vector<cv::Rect> bounding_box_B; // 鸟瞰图上的2D边界框列表
        std::vector<cv::Rect> bounding_box_D; // 深度图上的2D边界框列表（原始分辨率，未缩放）

        // 主要输出/发布的话题
        std::vector<box3D> box3Ds; // 相机坐标系下的3D边界框列表
        std::vector<box3D> box3DsWorld; // 世界坐标系下的3D边界框列表
        // vector<box3D> person_box3Ds; // 世界坐标系下人的3D边界框列表（已注释）

        // YOLO裁剪图像的偏移量
        int x0; // 来自YOLO的裁剪图像左上角的x坐标
        int y0; // 来自YOLO的裁剪图像左上角的y坐标

        // 测试变量
        int testx; // 测试用x坐标
        int testy; // 测试用y坐标
        int testby; // 测试用by坐标

        // 相机内参
        float fx; // x方向焦距（像素单位）
        float fy; // y方向焦距（像素单位）
        float px; // 主点x坐标（像素单位）
        float py; // 主点y坐标（像素单位）
        double depthScale_; // 深度缩放因子，真实深度 = 像素值 / depthScale_

        cv::Mat bird_view; // 鸟瞰图（俯视图），用于目标检测和跟踪
        UVtracker tracker; // 鸟瞰图空间的目标跟踪器

        /**
         * @brief 构造函数
         *
         * 初始化UV检测器，设置默认参数（相机内参、检测阈值等）
         */
        UVdetector();

        /**
         * @brief 读取深度图队列数据
         *
         * 从深度图队列中读取数据进行处理（可能用于多帧缓冲）
         *
         * @param depthq 深度图队列
         */
        void readdata(queue<cv::Mat> depthq);

        /**
         * @brief 读取深度图
         *
         * 由YOLO检测器调用，输入当前帧的深度图
         *
         * @param depth 输入的深度图（OpenCV Mat格式）
         */
        void readdepth(cv::Mat depth);

        /**
         * @brief 读取RGB图像
         *
         * 输入当前帧的RGB彩色图像，用于可视化或辅助检测
         *
         * @param RGB 输入的RGB图像（OpenCV Mat格式）
         */
        void readrgb(cv::Mat RGB);

        /**
         * @brief 提取U图
         *
         * 从深度图中提取U图（深度不连续性图）。
         * 通过计算深度梯度或差分检测边缘和障碍物边界。
         */
        void extract_U_map();

        /**
         * @brief 提取边界框
         *
         * 在U图上检测和提取目标的2D边界框。
         * 使用连通域分析或扫描线算法找到感兴趣区域。
         */
        void extract_bb();

        /**
         * @brief 提取鸟瞰图
         *
         * 将深度图投影到鸟瞰视角（俯视图），
         * 生成用于目标检测和跟踪的鸟瞰图表示。
         */
        void extract_bird_view();

        /**
         * @brief 执行检测
         *
         * 主检测函数，协调整个检测流程：
         * 提取U图 -> 检测边界框 -> 生成鸟瞰图 -> 提取3D边界框
         */
        void detect();

        /**
         * @brief 跟踪目标
         *
         * 调用跟踪器对检测到的目标进行跟踪，
         * 为每个目标分配ID并估计运动速度。
         */
        void track();

        /**
         * @brief 输出检测结果
         *
         * 将最终的3D边界框结果输出（可能发布ROS话题或返回结果）
         */
        void output();

        /**
         * @brief 显示深度图
         *
         * 可视化深度图，用于调试和展示
         */
        void display_depth();

        /**
         * @brief 提取3D边界框
         *
         * 从2D边界框和深度信息计算3D边界框的位置、尺寸和方向
         */
        void extract_3Dbox();

        // void display_RGB(); // 显示RGB图像（已注释）

        /**
         * @brief 显示U图
         *
         * 可视化U图，显示检测到的深度不连续性区域
         */
        void display_U_map();

        /**
         * @brief 将跟踪结果添加到鸟瞰图
         *
         * 在鸟瞰图上绘制跟踪结果（边界框、ID、轨迹等），
         * 用于可视化跟踪状态
         */
        void add_tracking_result();

        /**
         * @brief 显示鸟瞰图
         *
         * 可视化鸟瞰图及其上的检测和跟踪结果
         */
        void display_bird_view();


    };

    /**
     * @brief 合并两个UV边界框
     *
     * 将子边界框合并到父边界框中，更新合并后的边界框范围。
     * 用于在扫描线检测过程中合并相邻或重叠的检测区域。
     *
     * @param father 父边界框（将被扩展以包含子边界框）
     * @param son 子边界框（将被合并到父边界框中）
     * @return UVbox 合并后的新边界框，包含了两个输入边界框的并集区域
     */
    UVbox merge_two_UVbox(UVbox father, UVbox son);
}
#endif
