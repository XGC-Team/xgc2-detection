/*
    FILE: uvDetector.h
    ------------------
    UV检测器辅助类的函数定义
    实现基于深度图的U-V分散度检测算法，用于检测和跟踪三维空间中的动态障碍物
*/

#include <onboard_detector_lv/uvDetector.h>

// UVbox类实现
namespace onboardDetector{
    /**
     * @brief UVbox默认构造函数
     * 初始化一个空的UV边界框，所有参数设为默认值
     */
    UVbox::UVbox()
    {
        this->id = 0;                          // 分割ID初始化为0
        this->toppest_parent_id = 0;           // 最顶层父节点ID初始化为0
        this->bb = cv::Rect(cv::Point2f(0, 0), cv::Point2f(0, 0)); // 空边界框
    }

    /**
     * @brief UVbox参数化构造函数
     * @param seg_id 分割ID
     * @param row 行索引
     * @param left 左边界列索引
     * @param right 右边界列索引
     * 根据指定参数创建一个UV边界框对象
     */
    UVbox::UVbox(int seg_id, int row, int left, int right)
    {
        this->id = seg_id;                     // 设置分割ID
        this->toppest_parent_id = seg_id;      // 初始时最顶层父节点ID等于自身ID
        this->bb = cv::Rect(cv::Point2f(left, row), cv::Point2f(right, row)); // 创建边界框
    }

    /**
     * @brief 合并两个UVbox边界框
     * @param father 父边界框
     * @param son 子边界框
     * @return 合并后的边界框（继承父边界框的ID）
     * 通过计算两个边界框的最小外接矩形来实现合并
     */
    UVbox merge_two_UVbox(UVbox father, UVbox son)
    {
        // 合并边界框：计算包含两个框的最小外接矩形
        int top =       (father.bb.tl().y < son.bb.tl().y)?father.bb.tl().y:son.bb.tl().y;  // 取最小的y坐标（顶部）
        int left =      (father.bb.tl().x < son.bb.tl().x)?father.bb.tl().x:son.bb.tl().x; // 取最小的x坐标（左边）
        int bottom =    (father.bb.br().y > son.bb.br().y)?father.bb.br().y:son.bb.br().y; // 取最大的y坐标（底部）
        int right =     (father.bb.br().x > son.bb.br().x)?father.bb.br().x:son.bb.br().x; // 取最大的x坐标（右边）
        father.bb = cv::Rect(cv::Point2f(left, top), cv::Point2f(right, bottom));
        return father;
    }

    // UVtracker类实现 - 用于跟踪检测到的动态障碍物

    /**
     * @brief UVtracker默认构造函数
     * 初始化跟踪器参数
     */
    UVtracker::UVtracker()
    {
        this->overlap_threshold = 0.4;  // 设置边界框重叠阈值为0.4，用于判断是否为同一目标
    }

    /**
     * @brief 读取边界框数据并更新跟踪历史
     * @param now_bb 当前帧的鸟瞰图边界框
     * @param now_bb_D 当前帧的深度图边界框
     * @param box_3D 当前帧的3D边界框（引用传递，会被更新）
     *
     * 该函数负责：
     * 1. 更新所有历史记录（测量历史、3D框历史、滤波器、速度）
     * 2. 管理历史队列长度，保持在合理范围内
     */
    void UVtracker::read_bb(vector<cv::Rect> now_bb, vector<cv::Rect> now_bb_D, vector<box3D> &box_3D)
    {
        // 更新测量历史：将当前帧历史保存为前一帧历史
        this->pre_history = this->now_history;
        this->now_history.clear();
        this->now_history.resize(now_bb.size());  // 根据当前边界框数量调整大小

        // 更新3D边界框历史
        this->pre_box_3D_history = this->now_box_3D_history;
        this->now_box_3D_history.clear();
        this->now_box_3D_history.resize(now_bb.size());

        // 更新卡尔曼滤波器
        this->pre_filter = this->now_filter;
        this->now_filter.clear();
        this->now_filter.resize(now_bb.size());

        // 更新速度预测向量
        this->pre_V = this->now_V;
        this->now_V.clear();
        this->now_V.resize(now_bb.size());

        // 更新边界框数据
        this->pre_bb = this->now_bb;           // 保存上一帧边界框
        this->now_bb = now_bb;                 // 当前帧鸟瞰图边界框
        this->now_bb_D = now_bb_D;             // 当前帧深度图边界框
        this->now_box_3D = box_3D;             // 当前帧3D边界框


        // 维护历史队列长度
        for (size_t i=0 ; i<this->pre_box_3D_history.size() ; i++) {
            // 对于每个跟踪框，当历史记录超过10帧时进行队列维护
            if (this->pre_box_3D_history[i].size() > 10) {
                this->pre_box_3D_history[i].pop_front();  // 移除最旧的3D框记录
                // 注释掉的代码：可选择固定框的尺寸为历史平均值
                // this->now_box_3D[i].x_width = this->pre_box_3D_history[i].back().x_width;
                // this->now_box_3D[i].y_width = this->pre_box_3D_history[i].back().y_width;
                // this->now_box_3D[i].z_width = this->pre_box_3D_history[i].back().z_width;

                // 保持所有队列的同步，移除最旧的测量历史
                this->pre_history[i].erase(this->pre_history[i].begin());

            }
        }
        // 更新3D边界框输出
        box_3D = this->now_box_3D;
    }
    
    /**
     * @brief 检查跟踪状态，匹配当前帧和前一帧的边界框
     * @param box_3D 当前帧的3D边界框（引用传递）
     *
     * 跟踪匹配策略：
     * 1. 计算边界框重叠率
     * 2. 计算中心点距离
     * 3. 满足重叠阈值或距离阈值则认为是同一目标
     * 4. 匹配成功则继承历史信息，否则初始化新跟踪
     */
    void UVtracker::check_status(vector<box3D> &box_3D)
    {
        // 遍历当前帧的所有边界框
        for(size_t now_id = 0; now_id < this->now_bb.size(); now_id++)
        {
            bool tracked = false;  // 标记当前框是否匹配到前一帧的目标

            // 遍历前一帧的所有边界框，寻找匹配
            for(size_t pre_id = 0; pre_id < this->pre_bb.size(); pre_id++)
            {
                // 计算两个边界框的重叠区域
                cv::Rect overlap = this->now_bb[now_id] & this->pre_bb[pre_id];

                // 计算两个边界框中心点之间的欧氏距离
                float dist = std::sqrt( std::pow((this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width)-(this->pre_bb[pre_id].x + 0.5 * this->pre_bb[pre_id].width),2) + std::pow((this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height-(this->pre_bb[pre_id].y + 0.5 * this->pre_bb[pre_id].height)),2) );

                // 计算距离度量标准：两个框尺寸之和的平方根除以2
                float metric = std::sqrt( std::pow(this->now_bb[now_id].width+this->pre_bb[pre_id].width,2) + std::pow(this->now_bb[now_id].height+this->pre_bb[pre_id].height,2) )/2;

                // 匹配条件：重叠率超过阈值 或 中心距离小于度量标准
                if(max(overlap.area() / float(this->now_bb[now_id].area()), overlap.area() / float(this->pre_bb[pre_id].area())) >= this->overlap_threshold || dist<=metric)
                {
                    tracked = true;  // 标记为成功跟踪
                    // std::cout<<"tracked"<<std::endl;

                    // 继承历史轨迹信息
                    this->now_history[now_id] = this->pre_history[pre_id];
                    // 将当前帧中心点添加到轨迹历史
                    this->now_history[now_id].push_back(cv::Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
                    // 继承3D边界框历史
                    this->now_box_3D_history[now_id] = this->pre_box_3D_history[pre_id];

                    // 仅当目标完全在视野内时，才添加当前3D框到历史
                    // 边界条件：左上角(5,5)，右下角(635,475)，避免边缘检测不准确
                    if (this->now_bb_D[now_id].tl().x>5 && this->now_bb_D[now_id].tl().y>5 && this->now_bb_D[now_id].br().x<635 && this->now_bb_D[now_id].br().y<475) {
                        this->now_box_3D_history[now_id].push_back(this->now_box_3D[now_id]);
                    }
                    // 继承速度历史
                    this->now_V[now_id] = this->pre_V[pre_id];
                    // 继承卡尔曼滤波器状态
                    this->now_filter[now_id] = this->pre_filter[pre_id];
                    // MatrixXd z(6,1); // 测量向量（已注释）

                    break;  // 找到匹配后跳出循环
                }

            }
            // 如果没有找到匹配的前一帧目标
            if(!tracked)
            {
                // std::cout<<now_id<<" LOSS TRACK\n"<<std::endl;
                // 将当前检测添加到历史记录，作为新目标
                this->now_history[now_id].push_back(cv::Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));

                // 初始化速度为零
                Eigen::MatrixXd V(2,1);
                V << 0.0, 0.0;
                this->now_V[now_id].push_back(V);

                // 同样只在视野内时添加3D框历史
                if (this->now_bb_D[now_id].tl().x>5 && this->now_bb_D[now_id].tl().y>5 && this->now_bb_D[now_id].br().x<635 && this->now_bb_D[now_id].br().y<475) {
                    this->now_box_3D_history[now_id].push_back(this->now_box_3D[now_id]);
                }

            }
        }
    }  

    // UVdetector类实现 - 核心检测器

    /**
     * @brief UVdetector默认构造函数
     * 初始化所有检测参数和相机内参
     */
    UVdetector::UVdetector()
    {
        // U-V图构建参数
        this->row_downsample = 4;           // 行下采样因子，将深度图沿深度方向压缩
        this->col_scale = 0.5;              // 列缩放因子，将图像宽度缩小到原来的一半

        // 深度范围参数
        this->min_dist = 10;                // 最小检测距离：10mm
        this->max_dist = 8000;              // 最大检测距离：8000mm（8米）

        // 分割阈值参数
        this->threshold_point = 3;          // 点阈值：每个深度bin至少需要3个点
        this->threshold_line = 2;           // 线阈值：线的累积强度阈值
        this->min_length_line = 6;          // 最小线长度：至少6个像素

        // 可视化开关
        this->show_bounding_box_U = true;   // 显示U图边界框
        this->show_bounding_box_U = true;   // 显示U图边界框（重复赋值）

        // 相机内参矩阵参数（可从/camera/.../camera_info话题获取）
        this->fx = 608.08740234375;         // x方向焦距
        this->fy = 608.1791381835938;       // y方向焦距
        this->px = 317.48284912109375;      // x方向主点坐标
        this->py = 234.11557006835938;      // y方向主点坐标

        // 初始化参考点坐标
        this->x0 = 0;
        this->y0 = 0;
    }

    /**
     * @brief 从深度图队列读取数据
     * @param depthq 深度图队列
     *
     * 通过取队列首尾深度图的逐像素最大值来合成深度图，
     * 可以填补部分深度缺失的区域
     */
    void UVdetector::readdata(queue<cv::Mat> depthq)
    {
        // 取队列首尾两帧的逐像素最大值，减少深度缺失
        this->depth = max(depthq.front(), depthq.back());
        double minVal;   // 最小深度值
        double maxVal;   // 最大深度值
        cv::Point minLoc; // 最小值位置
        cv::Point maxLoc; // 最大值位置
        // resize(this->depth, this->depth, cv::Size(50,50)); // 调整大小（已注释）
        minMaxLoc( this->depth, &minVal, &maxVal, &minLoc, &maxLoc );  // 获取深度图统计信息
    }

    /**
     * @brief 读取单帧深度图
     * @param depth 输入深度图
     * 直接读取深度图并计算其统计信息
     */
    void UVdetector::readdepth(cv::Mat depth){
        this->depth = depth;
        double minVal;   // 最小深度值
        double maxVal;   // 最大深度值
        cv::Point minLoc; // 最小值位置
        cv::Point maxLoc; // 最大值位置

        minMaxLoc( this->depth, &minVal, &maxVal, &minLoc, &maxLoc ); // 获取深度图统计信息
    }

    /**
     * @brief 读取RGB图像
     * @param RGB 输入RGB图像
     * 读取RGB图像并调整到标准尺寸720x400
     */
    void UVdetector::readrgb(cv::Mat RGB)
    {
        this->RGB = RGB;
        resize(this->RGB, this->RGB, cv::Size(720,400));  // 调整到标准尺寸
        // imshow("RGB", this->RGB); // 显示RGB图像（已注释）
    }

    /**
     * @brief 从深度图提取U-V分散度图
     *
     * U-V分散度算法核心：
     * - U轴：深度方向，将深度范围离散化为若干bins
     * - V轴：图像列方向
     * - U_map[u,v]：在列v、深度bin u处的点数量
     *
     * 算法步骤：
     * 1. 对深度图进行缩放处理
     * 2. 将深度值映射到深度bins
     * 3. 统计每个(bin, col)位置的点数
     * 4. 高斯平滑以减少噪声
     */
    void UVdetector::extract_U_map()
    {
        // 对深度图进行列方向缩放，减少计算量
        cv::Mat depth_rescale;
        resize(this->depth, depth_rescale, cv::Size(),this->col_scale , 1);  // 宽度缩放，高度保持不变
        cv::Mat depth_low_res_temp = cv::Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);

        // 构建深度掩码
        cv::Rect mask_depth;
        uint8_t histSize = this->depth.rows / this->row_downsample;  // 深度方向bin的数量

        // 计算每个bin的宽度（深度范围）
        int bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));

        // 初始化U_map：行数为深度bins数量，列数为缩放后的图像宽度
        this->U_map = cv::Mat::zeros(histSize, depth_rescale.cols, CV_8UC1);

        int depth_rescale_val = 0;

        // 遍历缩放后深度图的所有像素
        for(int col = 0; col < depth_rescale.cols; col++)
        {
            for(int row = 0; row < depth_rescale.rows; row++)
            {
                // printf("HERE %d,%d",row,col);
                // 将深度值从原始单位转换为毫米
                depth_rescale_val = int((float(depth_rescale.at<unsigned short>(row, col))/this->depthScale_)*1000.0);
                // printf("raw depth %d, %d: %d\n",row,col,depth_rescale_val);

                // 仅处理有效深度范围内的点
                if(depth_rescale_val > this->min_dist && depth_rescale_val < this->max_dist)
                {
                    // 计算该深度值对应的bin索引
                    uint8_t bin_index = (depth_rescale_val - this->min_dist) / bin_width;
                    depth_low_res_temp.at<uchar>(row, col) = bin_index;
                    // printf("depth val %d",depth_rescale_val);

                    // 在U_map对应位置累加计数（防止溢出）
                    if(this->U_map.at<uchar>(bin_index, col) < 255)
                    {
                        // printf("here %d",this->U_map.at<uchar>(bin_index, col));
                        this->U_map.at<uchar>(bin_index, col) ++;  // 该(bin, col)位置点数+1
                    }
                }
            }
        }
        this->depth_low_res = depth_low_res_temp;

        // 对U_map进行高斯平滑，减少噪声影响
        GaussianBlur(this->U_map, this->U_map, cv::Size(5,9), 10, 10);
        // printf("rescaled depth map: %d", depth_rescale.at<unsigned short>(testy,testx));
    }

    /**
     * @brief 从U_map提取边界框
     *
     * 算法流程：
     * 1. 逐行扫描U_map，寻找连续的高强度线段
     * 2. 将满足条件的线段标记为分割片段
     * 3. 合并相邻行的重叠片段（类似连通域分析）
     * 4. 最终得到完整的边界框
     *
     * 使用Union-Find思想进行分割片段的合并
     */
    void UVdetector::extract_bb()
    {
        // 初始化掩码矩阵，用于记录每个像素属于哪个分割ID
        std::vector<vector<int> > mask(this->U_map.rows, vector<int>(this->U_map.cols, 0));

        // 初始化参数
        int u_min = this->threshold_point * this->row_downsample;  // 点数最小阈值
        int sum_line = 0;        // 当前线段的累积强度
        int max_line = 0;        // 当前线段的最大强度
        int length_line = 0;     // 当前线段的长度
        int seg_id = 0;          // 分割ID计数器
        std::vector<UVbox> UVboxes;  // 存储所有检测到的UV边界框

        // printf("Umap rows %d\n",this->U_map.rows);
        // 逐行扫描U_map
        for(int row = 0; row < this->U_map.rows; row++)
        {
            for(int col = 0; col < this->U_map.cols; col++)
            {
                // 判断当前点是否为感兴趣点（强度足够高）
                if(this->U_map.at<uchar>(row,col) >= u_min) // 该深度处的点数 >= u_min
                {
                    // 更新当前线段信息
                    length_line++;  // 线段长度+1
                    sum_line += this->U_map.at<uchar>(row,col);  // 累加强度
                    max_line = (this->U_map.at<uchar>(row,col) > max_line)?this->U_map.at<uchar>(row,col):max_line;  // 更新最大强度
                }
                // 线段结束条件：当前点强度不足 或 到达行尾
                if(this->U_map.at<uchar>(row,col) < u_min || col == this->U_map.cols - 1)
                {
                    // 处理行尾边界情况
                    col = (col == this->U_map.cols - 1)? col + 1:col;

                    // 判断当前线段是否为有效候选（长度和强度都满足要求）
                    if(length_line > this->min_length_line && sum_line > this->threshold_line * max_line)
                    {
                        seg_id++;  // 分配新的分割ID
                        UVboxes.push_back(UVbox(seg_id, row, col - length_line, col - 1));

                        // 在掩码中标记该线段的分割ID
                        for(int c = col - length_line; c < col - 1; c++)
                        {
                            mask[row][c] = seg_id;
                        }

                        // 如果不是第一行，需要与上一行的分割进行合并
                        if(row != 0)
                        {
                            // 遍历当前线段，检查是否与上一行有重叠
                            for(int c = col - length_line; c < col - 1; c++)
                            {
                                if(mask[row - 1][c] != 0)  // 上一行该位置有分割
                                {
                                    // Union-Find合并操作：更新最顶层父节点ID
                                    // 初始时，toppest_parent_id就是自身的seg_id
                                    if(UVboxes[mask[row - 1][c] - 1].toppest_parent_id < UVboxes.back().toppest_parent_id)
                                    {
                                        // 采用上一行分割的父节点ID
                                        UVboxes.back().toppest_parent_id = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                    }
                                    else
                                    {
                                        // 需要更新所有相关分割的父节点ID
                                        int temp = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                        for(size_t b = 0; b < UVboxes.size(); b++)
                                        {
                                            UVboxes[b].toppest_parent_id = (UVboxes[b].toppest_parent_id == temp)?UVboxes.back().toppest_parent_id:UVboxes[b].toppest_parent_id;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // 重置线段统计信息
                    sum_line = 0;
                    max_line = 0;
                    length_line = 0;
                }
            }
        }

        // 将线段分组为边界框
        this->bounding_box_U.clear();

        // 合并具有相同最顶层父节点的所有边界框
        for(size_t b = 0; b < UVboxes.size(); b++)
        {
            // 如果当前框就是根节点（自己是自己的父节点）
            if(UVboxes[b].id == UVboxes[b].toppest_parent_id)
            {
                // 遍历后续所有框，找到属于同一组的子框
                for(size_t s = b + 1; s < UVboxes.size(); s++)
                {
                    if(UVboxes[s].toppest_parent_id == UVboxes[b].id)
                    {
                        // 合并子框到父框
                        UVboxes[b] = merge_two_UVbox(UVboxes[b], UVboxes[s]);

                    }
                }
                // 检查合并后边界框的大小，过滤掉太小的框
                if(UVboxes[b].bb.area() >= 25)  // 面积阈值：25像素
                {
                    this->bounding_box_U.push_back(UVboxes[b].bb);
                    // printf("bbox_U [b] %f",UVboxes[b].bb.tl().y);
                }
            }
        }
    }

    /**
     * @brief 主检测函数
     * 按顺序执行完整的检测流程
     */
    void UVdetector::detect()
    {
        // 步骤1：从深度图提取U-V分散度图
        this->extract_U_map();

        // 步骤2：从U_map提取边界框
        this->extract_bb();

        // 步骤3：提取鸟瞰图边界框
        this->extract_bird_view();

        // 步骤4：提取物体高度（已注释）
        // this->extract_height();
    }  
    /**
     * @brief 显示深度图及检测结果
     * 将深度图归一化到0-255范围并应用伪彩色，叠加检测到的边界框
     */
    void UVdetector::display_depth()
    {
        // 为了更好的可视化，需要将深度图归一化到(0, 255)范围
        cv::Mat depth_normalized;
        this->depth.copyTo(depth_normalized);
        double min, max;
        cv::minMaxIdx(depth_normalized, &min, &max);  // 找到最小最大值
        cv::convertScaleAbs(depth_normalized, depth_normalized, 255. / max);  // 归一化
        depth_normalized.convertTo(depth_normalized, CV_8UC1);  // 转换为8位单通道
        applyColorMap(depth_normalized, depth_normalized, cv::COLORMAP_BONE);  // 应用骨骼伪彩色映射

        // 在深度图上绘制所有检测到的边界框
        for (size_t i=0;i<this->bounding_box_D.size();i++){
            rectangle(depth_normalized, bounding_box_D[i], cv::Scalar(0, 255, 0), 5, 8, 0);  // 绿色边界框
        }
        this->depth_show = depth_normalized;
        // imshow("Depth", depth_normalized);  // 显示深度图（已注释）
        // waitKey(1);
    }

    /**
     * @brief 提取3D边界框
     *
     * 功能：
     * 1. 根据U_map中的边界框，在深度图中找到对应的3D区域
     * 2. 计算3D边界框在相机坐标系下的位置和尺寸
     * 3. 将图像坐标转换为世界坐标（单位：米）
     *
     * 坐标系转换：
     * - 图像坐标系：原点在图像左上角
     * - 相机坐标系：x轴指向右，y轴指向下，z轴指向前方（深度方向）
     * - 世界坐标系：x = 深度方向，y = -图像x方向，z = -图像y方向
     */
    void UVdetector::extract_3Dbox()
    {
        // 该函数返回世界坐标系下的3D边界框，用于发布
        cv::Mat depth_resize;
        resize(depth, depth_resize, cv::Size(), this->col_scale, 1);  // 缩放深度图
        float histSize = this->depth.rows / this->row_downsample;
        // printf("rows, ros_downsmaple %d, %d\n", this->depth.rows, this->row_downsample);
        float bin_width = ceil((this->max_dist - this->min_dist) / histSize);  // 深度bin宽度
        // printf("max, min, hist size , bin_width %d, %d, %f, %f\n",this->max_dist, this->min_dist, histSize, bin_width);

        // 边界框参数
        int x;              // 边界框左边界
        int y_up;           // 边界框上边界
        int y_down;         // 边界框下边界
        int width;          // 边界框宽度
        int bin_index_small;  // 最近深度bin索引
        int bin_index_large;  // 最远深度bin索引
        float depth_in_near;  // 最近深度值
        float depth_of_depth; // 深度厚度
        float depth_in_far;   // 最远深度值
        float depth_resize_val; // 当前深度值

        // 图像帧坐标
        size_t im_frame_x;        // 中心x坐标
        size_t im_frame_x_width;  // x方向宽度
        size_t im_frame_y;        // 中心y坐标
        size_t im_frame_y_width;  // y方向高度

        // GaussianBlur(depth_resize, depth_resize, cv::Size(5,9), 0, 0);  // 可选的高斯平滑

        // 连续性检查参数：确保检测到的是连续的障碍物表面
        int num_check  = 15;  // 向下检查15个点以验证连续性

        this->box3Ds.clear();              // 清空3D边界框列表
        this->bounding_box_D.clear();      // 清空深度图边界框列表

        // 遍历所有U_map边界框，提取对应的3D信息
        for (size_t b = 0; b < this->bounding_box_U.size(); b++) {
            // 从U_map边界框获取x和宽度信息
            x = this->bounding_box_U[b].tl().x;        // 左边界
            width = this->bounding_box_U[b].width;     // 宽度

            // 初始化y方向的搜索范围
            y_up = depth_resize.rows;  // 初始化为图像底部
            // std::cout<<" y_up init "<<y_up<<std::endl;
            y_down = 0;                // 初始化为图像顶部

            // 获取深度bin索引范围
            bin_index_small = this->bounding_box_U[b].tl().y;  // 最近深度bin
            bin_index_large = this->bounding_box_U[b].br().y;  // 最远深度bin

            // 计算实际深度范围（毫米）
            depth_in_near = (bin_index_small * bin_width + this->min_dist);      // 最近深度
            depth_of_depth = (bin_index_large - bin_index_small) * bin_width;   // 深度厚度
            depth_in_far = depth_of_depth*1.3 + depth_in_near;  // 最远深度，乘以1.3考虑遮挡导致的深度截断
            // printf("bin_s, bin_l, near, far %d, %d, %f, %f\n",bin_index_small, bin_index_large, depth_in_near, depth_in_far);

            // 在边界框范围内搜索障碍物的y方向边界
            for (int i = x ; i < x + width; i++) { // 遍历边界框内的若干列
                for (int j = 0; j < depth_resize.rows - 1; j++) { // 遍历每一行
                    // 获取当前点的深度值（毫米）
                    depth_resize_val = (float(depth_resize.at<unsigned short>(j,i))/this->depthScale_)*1000.0;

                    // 如果深度值在有效范围内
                    if (float(depth_resize_val) >= depth_in_near && depth_resize_val <= depth_in_far) {
                        // 向下检查num_check个点，确保深度连续性
                        for (int check = 0; check < num_check; check++) {
                            depth_resize_val = (float(depth_resize.at<unsigned short>(j + check + 1,i))/this->depthScale_)*1000.0;
                            // 如果发现深度不连续，说明不是有效的障碍物表面
                            if (depth_resize_val < depth_in_near ||
                            depth_resize_val > depth_in_far) {
                                // 深度不连续，跳出检查
                                break;
                            }
                            // 如果所有点都通过检查
                            if (check == num_check-1) {
                                // 更新y方向边界
                                if (y_up > j) y_up = j;      // 更新上边界（最小y）
                                if (y_down < j) y_down = j;  // 更新下边界（最大y）
                            }
                        }
                    }
                }
            }

            // 保存深度图中的边界框（需要将坐标从缩放图还原到原始深度图）
            float bb_x = x / this->col_scale;               // x坐标还原
            float bb_width = width / this->col_scale;       // 宽度还原
            float bb_y = y_up;                              // y坐标保持不变
            // std::cout<<" y_up  "<<y_up<<" y_down "<<y_down<<std::endl;
            float bb_height = y_down-y_up;                  // 高度
            this->bounding_box_D.push_back(cv::Rect(bb_x, bb_y, bb_width, bb_height));

            box3D curr_box;  // 当前3D边界框

            // 计算边界框在原始深度图中的中心坐标和尺寸
            im_frame_x  = (x + width / 2) / this->col_scale;  // x方向中心（还原到原始尺度）
            im_frame_x_width = width / this->col_scale;        // x方向宽度

            int Y_w = (depth_in_near + depth_in_far) / 2;     // 中心深度值（毫米）
            im_frame_y = (y_down + y_up) / 2;                  // y方向中心
            im_frame_y_width = y_down - y_up;                  // y方向高度

            // printf("im_frame box %d x: %d, y:%d, x_width %d, y_width %d, Y_W %d\n",b,im_frame_x, im_frame_y, im_frame_x_width, im_frame_y_width, Y_w);

            testx = im_frame_x;
            testy = im_frame_y;
            testby = bin_index_small;

            // 图像坐标系到世界坐标系的转换
            // 世界坐标系定义：
            // - x轴：深度方向（相机z轴）
            // - y轴：图像x轴的负方向
            // - z轴：图像y轴的负方向

            // 使用针孔相机模型进行坐标转换
            curr_box.x =  (im_frame_x-this->px)*Y_w/this->fx;  // x = (u - cx) * Z / fx
            curr_box.y = (im_frame_y-this->py)*Y_w/this->fy;   // y = (v - cy) * Z / fy
            curr_box.x_width = im_frame_x_width*Y_w/this->fx;  // 宽度转换
            curr_box.y_width = im_frame_y_width*Y_w/this->fy;  // 高度转换
            curr_box.z = Y_w;                                   // 深度（z方向）
            curr_box.z_width = depth_in_far-depth_in_near;     // 深度方向厚度
            // std::cout<<"imFrameYWidth "<<im_frame_y_width<<" Y_W "<<Y_w<<" fy "<<this->fy<<std::endl;

            // 从毫米转换为米
            curr_box.x /=1000.0;
            curr_box.y /=1000.0;
            curr_box.z /=1000.0;
            curr_box.x_width /=1000.0;
            curr_box.y_width /=1000.0;
            curr_box.z_width /=1000.0;
            // std::cout<<"uv box on came raw z_width "<<curr_box.z_width<<std::endl;
            box3Ds.push_back(curr_box);
            // printf("3d box %d on cam: %f, %f, %f,%f ,%f, %f\n",curr_box.x,curr_box.y,curr_box.z,curr_box.x_width,curr_box.y_width,curr_box.z_width);
            // printf("depth in near: %f \n",depth_in_near);
        }
    }
        
    // void UVdetector::display_RGB()
    // {
    //     // RGB图像显示函数（未实现）
    // }

    /**
     * @brief 显示U-V分散度图
     * 将U_map可视化，应用伪彩色映射并叠加检测到的边界框
     */
    void UVdetector::display_U_map()
    {
        // 如果需要显示边界框
        if(this->show_bounding_box_U)
        {
            this->U_map = this->U_map * 10;  // 增强对比度
            this->U_map_show = this->U_map;

            double min, max;
            cv::minMaxIdx(this->U_map_show, &min, &max);  // 获取最小最大值
            cvtColor(this->U_map_show, this->U_map_show, cv::COLOR_GRAY2RGB);  // 转换为RGB
            cv::convertScaleAbs(this->U_map_show, this->U_map_show, 255./ max);  // 归一化
            this->U_map_show.convertTo(this->U_map_show, CV_8UC1);  // 转换为8位
            applyColorMap(this->U_map_show, this->U_map_show, cv::COLORMAP_JET);  // 应用jet伪彩色

            // 绘制所有边界框
            for(size_t b = 0; b < this->bounding_box_U.size(); b++)
            {
                // 将边界框高度扩展2倍用于显示（因为深度方向被压缩）
                cv::Rect final_bb = cv::Rect(this->bounding_box_U[b].tl(),cv::Size(this->bounding_box_U[b].width, 2 * this->bounding_box_U[b].height));
                rectangle(this->U_map_show, final_bb, cv::Scalar(0, 255, 0), 1, 8, 0);  // 绿色边界框

                // 可选：绘制中心点（已注释）
                // circle(this->U_map_show, cv::Point2f(this->bounding_box_U[b].tl().x + 0.5 * this->bounding_box_U[b].width, this->bounding_box_U[b].br().y ), 2, cv::Scalar(0, 0, 255), 5, 8, 0);
            }
        }
        // imshow("U map", this->U_map_show);  // 显示U图（已注释）
        // waitKey(1);
    }

    /**
     * @brief 提取鸟瞰图边界框
     *
     * 将U_map中的边界框转换到鸟瞰图坐标系
     * 鸟瞰图：从上往下俯视的2D投影
     * - x轴：水平方向（图像x方向在世界中的投影）
     * - y轴：深度方向（远离相机的方向）
     *
     * 单位说明：鸟瞰图中使用厘米作为单位（除以10）
     * x, y是边界框左下角点的坐标
     */
    void UVdetector::extract_bird_view()
    {
        // 提取鸟瞰图边界框
        uint8_t histSize = this->depth.rows / this->row_downsample;
        uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
        this->bounding_box_B.clear();
        this->bounding_box_B.resize(this->bounding_box_U.size());

        for(size_t b = 0; b < this->bounding_box_U.size(); b++)
        {
            // U_map边界框 -> 鸟瞰图边界框转换

            // 计算边界框的深度（最远点深度，单位：厘米）
            float bb_depth = this->bounding_box_U[b].br().y * bin_width / 10;

            // 根据透视投影计算实际宽度（厘米）
            float bb_width = bb_depth * this->bounding_box_U[b].width / this->fx;

            // 深度方向的高度（厘米）
            float bb_height = this->bounding_box_U[b].height * bin_width / 10;

            // 计算x坐标（相对于相机中心的偏移）
            float bb_x = bb_depth * (this->bounding_box_U[b].tl().x / this->col_scale - this->px) / this->fx;

            // 计算y坐标（深度方向）
            // 假设最远深度值是中心点的深度，检测到的深度差是整个物体的深度范围
            float bb_y = bb_depth - 0.5 * bb_height;  // y轴是深度方向

            this->bounding_box_B[b] = cv::Rect(bb_x, bb_y, bb_width, bb_height);
        }

        // 初始化鸟瞰图画布（500x1000像素）
        this->bird_view = cv::Mat::zeros(500, 1000, CV_8UC1);
        cvtColor(this->bird_view, this->bird_view, cv::COLOR_GRAY2RGB);  // 转换为RGB用于彩色绘制
    }

    /**
     * @brief 显示鸟瞰图
     *
     * 在鸟瞰图上绘制：
     * 1. 相机视野边界线（左右两条绿线）
     * 2. 检测到的边界框（红色矩形）
     * 3. 边界框中心点（红色圆点）
     *
     * 坐标系：原点在图像底部中心（相机位置）
     */
    void UVdetector::display_bird_view()
    {
        // 相机位置（图像底部中心）
        cv::Point2f center = cv::Point2f(this->bird_view.cols / 2, this->bird_view.rows);

        // 计算左边界线的终点（相机视野左边缘在最远处的投影）
        cv::Point2f left_end_to_center = cv::Point2f( this->bird_view.rows * (0 - this->px) / this->fx, -this->bird_view.rows);

        // 计算右边界线的终点（相机视野右边缘在最远处的投影）
        cv::Point2f right_end_to_center = cv::Point2f( this->bird_view.rows * (this->depth.cols - this->px) / this->fx, -this->bird_view.rows);

        // 绘制视野边界的两条侧线（绿色）
        line(this->bird_view, center, center + left_end_to_center, cv::Scalar(0, 255, 0), 3, 8, 0);
        line(this->bird_view, center, center + right_end_to_center, cv::Scalar(0, 255, 0), 3, 8, 0);

        // 绘制所有检测到的边界框
        for(size_t b = 0; b < this->bounding_box_U.size(); b++)
        {
            // 坐标系转换：从世界坐标转换到显示坐标
            cv::Rect final_bb = this->bounding_box_B[b];
            final_bb.y = center.y - final_bb.y - final_bb.height;  // y轴翻转（OpenCV y轴向下，鸟瞰图y轴向上）
            final_bb.x = final_bb.x + center.x;  // x轴平移到图像中心

            // 计算边界框中心点
            cv::Point2f bb_center = cv::Point2f(final_bb.x + 0.5 * final_bb.width, final_bb.y + 0.5 * final_bb.height);

            // 绘制边界框（红色矩形）
            rectangle(this->bird_view, final_bb, cv::Scalar(0, 0, 255), 3, 8, 0);

            // 绘制中心点（红色圆点）
            circle(this->bird_view, bb_center, 3, cv::Scalar(0, 0, 255), 5, 8, 0);
        }

        // 缩小显示尺寸为原来的50%
        resize(this->bird_view, this->bird_view, cv::Size(), 0.5, 0.5);
        // imshow("Bird's View", this->bird_view);  // 显示鸟瞰图（已注释）
        // waitKey(1);
    }

    /**
     * @brief 在鸟瞰图上添加跟踪结果
     *
     * 绘制内容：
     * 1. 估计的目标中心位置（绿色圆点）
     * 2. 估计的边界框（绿色矩形）
     * 3. 速度矢量（白色箭头）
     * 4. 历史轨迹（红色折线）
     */
    void UVdetector::add_tracking_result()
    {
        cv::Point2f center = cv::Point2f(this->bird_view.cols / 2, this->bird_view.rows);

        // 遍历所有跟踪目标
        for(size_t b = 0; b < this->tracker.now_bb.size(); b++)
        {
            // 从卡尔曼滤波器获取估计的中心位置并转换坐标
            cv::Point2f estimated_center = cv::Point2f(this->tracker.now_filter[b].output(0), this->tracker.now_filter[b].output(1));
            estimated_center.y = center.y - estimated_center.y;  // y轴翻转
            estimated_center.x = estimated_center.x + center.x;  // x轴平移

            // 绘制估计的中心点（绿色圆点）
            circle(this->bird_view, estimated_center, 5, cv::Scalar(0, 255, 0), 5, 8, 0);

            // 从滤波器获取边界框尺寸并绘制（绿色矩形）
            cv::Point2f bb_size = cv::Point2f(this->tracker.now_filter[b].output(4), this->tracker.now_filter[b].output(5));
            rectangle(this->bird_view, cv::Rect(estimated_center - 0.5 * bb_size, estimated_center + 0.5 * bb_size), cv::Scalar(0, 255, 0), 3, 8, 0);

            // 绘制速度矢量（白色箭头）
            cv::Point2f velocity = cv::Point2f(this->tracker.now_filter[b].output(2), this->tracker.now_filter[b].output(3));
            velocity.y = -velocity.y;  // y方向在鸟瞰图中与OpenCV默认方向相反
            // printf("velocity in birdview 10mm/s: %f, %f , center x ,y: %f, %f, bbox size x, y:%f, %f\n", velocity.x, -velocity.y, estimated_center.x, estimated_center.y, bb_size.x, bb_size.y);
            line(this->bird_view, estimated_center, estimated_center + velocity, cv::Scalar(255, 255, 255), 3, 8, 0);

            // 绘制历史轨迹（红色折线）
            for(size_t h = 1; h < this->tracker.now_history[b].size(); h++)
            {
                // 轨迹线段的起点
                cv::Point2f start = this->tracker.now_history[b][h - 1];
                start.y = center.y - start.y;
                start.x = start.x + center.x;

                // 轨迹线段的终点
                cv::Point2f end = this->tracker.now_history[b][h];
                end.y = center.y - end.y;
                end.x = end.x + center.x;

                // 绘制轨迹线段（红色）
                line(this->bird_view, start, end, cv::Scalar(0, 0, 255), 3, 8, 0);
            }
        }
    }
    
    /**
     * @brief 跟踪函数
     *
     * 执行完整的跟踪流程：
     * 1. 读取当前帧的边界框数据
     * 2. 检查跟踪状态（数据关联）
     * 3. 在鸟瞰图上可视化跟踪结果
     */
    void UVdetector::track()
    {
        // float before = this->box3Ds[0].x_width;  // 用于验证（已注释）

        // 读取边界框数据并更新跟踪器
        this->tracker.read_bb(this->bounding_box_B, this->bounding_box_D, this->box3Ds);

        // 检查跟踪状态，进行数据关联
        this->tracker.check_status(this->box3Ds);

        // 在鸟瞰图上添加跟踪结果可视化
        this->add_tracking_result();

        // float after = this->box3Ds[0].x_width;  // 用于验证（已注释）
        // printf("verify : %f, %f", before, after);
    }
}  // namespace onboardDetector
