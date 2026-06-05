/*
	文件名: kalman_filter.cpp
	--------------------------------------
	卡尔曼滤波器速度估计器的函数定义

	该文件实现了标准卡尔曼滤波器，用于对动态目标的状态进行最优估计。
	卡尔曼滤波器通过预测-更新两步循环，融合系统模型和传感器测量，
	在存在噪声的情况下实现对系统状态的最优估计。

	核心算法步骤：
	1. 预测步骤：基于系统动力学模型预测下一时刻的状态和协方差
	2. 更新步骤：基于传感器测量修正预测状态，计算卡尔曼增益
*/
#include <onboard_detector_lv/kalmanFilter.h>
using Eigen::MatrixXd;

namespace onboardDetector{
    /**
     * @brief 卡尔曼滤波器构造函数
     *
     * 初始化卡尔曼滤波器对象，将初始化标志设为false。
     * 在调用setup函数配置滤波器参数之前，滤波器不可用。
     */
    kalman_filter::kalman_filter()
    {
        this->is_initialized = false;  // 标记滤波器尚未初始化
    }

    /**
     * @brief 设置卡尔曼滤波器的初始参数
     *
     * @param states 初始状态向量 (n×1)，包含系统的初始状态估计
     * @param A 状态转移矩阵 (n×n)，描述系统动力学模型 x(k+1) = A*x(k) + B*u(k)
     * @param B 控制输入矩阵 (n×m)，描述控制输入如何影响状态
     * @param H 观测矩阵 (m×n)，描述状态到观测的映射关系 z(k) = H*x(k)
     * @param P 状态估计协方差矩阵 (n×n)，表示状态估计的不确定性
     * @param Q 过程噪声协方差矩阵 (n×n)，描述系统模型的不确定性
     * @param R 测量噪声协方差矩阵 (m×m)，描述传感器测量的不确定性
     *
     * 该函数初始化卡尔曼滤波器的所有核心参数，使滤波器进入可用状态。
     */
    void kalman_filter::setup(const MatrixXd& states, const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, const MatrixXd& P, const MatrixXd& Q, const MatrixXd& R)
    {
        this->states = states;  // 设置初始状态向量
        this->A = A;            // 设置状态转移矩阵
        this->B = B;            // 设置控制输入矩阵
        this->H = H;            // 设置观测矩阵
        this->P = P;            // 设置初始状态协方差矩阵
        this->Q = Q;            // 设置过程噪声协方差矩阵
        this->R = R;            // 设置测量噪声协方差矩阵
        this->is_initialized = true;  // 标记滤波器已完成初始化
    }

    /**
     * @brief 更新状态转移矩阵
     *
     * @param A 新的状态转移矩阵 (n×n)
     *
     * 该函数用于动态更新系统的状态转移矩阵A，适用于时变系统或需要
     * 在运行时调整系统模型的场景。例如，对于离散化的连续系统，
     * 当采样时间dt变化时，需要重新计算状态转移矩阵。
     */
    void kalman_filter::setA(const MatrixXd& A)
    {
        this->A = A;  // 更新状态转移矩阵
    }

    /**
     * @brief 执行卡尔曼滤波的预测-更新循环
     *
     * @param z 当前时刻的观测向量 (m×1)，来自传感器的测量值
     * @param u 当前时刻的控制输入向量 (l×1)，系统的控制量
     *
     * 该函数实现了标准卡尔曼滤波器的核心算法，包含两个关键步骤：
     * 1. 预测步骤（Prediction）：基于系统模型预测先验状态估计
     * 2. 更新步骤（Update）：基于观测值修正后验状态估计
     *
     * 算法流程：
     * 预测阶段：
     *   x̂(k|k-1) = A*x̂(k-1|k-1) + B*u(k)       - 状态预测
     *   P(k|k-1) = A*P(k-1|k-1)*A^T + Q         - 协方差预测
     * 更新阶段：
     *   S(k) = H*P(k|k-1)*H^T + R               - 新息协方差矩阵
     *   K(k) = P(k|k-1)*H^T*S(k)^(-1)           - 卡尔曼增益
     *   x̂(k|k) = x̂(k|k-1) + K*(z(k) - H*x̂(k|k-1)) - 状态更新
     *   P(k|k) = (I - K*H)*P(k|k-1)             - 协方差更新
     */
    void kalman_filter::estimate(const MatrixXd& z, const MatrixXd& u)
    {
        // ========== 预测步骤 (Prediction Step) ==========
        // 基于系统动力学模型和控制输入预测下一时刻的状态
        this->states = this->A * this->states + this->B * u;  // 先验状态估计: x̂(k|k-1) = A*x̂(k-1) + B*u

        // 预测状态估计的协方差矩阵，包含过程噪声的影响
        this->P = this->A * this->P * this->A.transpose() + this->Q;  // 先验协方差估计: P(k|k-1) = A*P*A^T + Q

        // cout << "prediction: " << endl;
        // cout << this->states << endl;

        // ========== 更新步骤 (Update Step) ==========
        // 计算新息（innovation）协方差矩阵S，表示测量预测的不确定性
        MatrixXd S = this->R + this->H * this->P * this->H.transpose();  // S = H*P*H^T + R，新息协方差

        // 计算卡尔曼增益K，决定了预测值和测量值的权重分配
        // K值越大，越信任测量值；K值越小，越信任模型预测
        MatrixXd K = this->P * this->H.transpose() * S.inverse();  // K = P*H^T*S^(-1)，卡尔曼增益

        // 基于测量值修正状态估计，(z - H*states)为新息（测量残差）
        this->states = this->states + K * (z - this->H * this->states);  // 后验状态估计: x̂(k|k) = x̂(k|k-1) + K*y

        // 更新状态估计的协方差矩阵，反映融合测量信息后不确定性的降低
        this->P = (MatrixXd::Identity(this->P.rows(),this->P.cols()) - K * this->H) * this->P;  // 后验协方差: P(k|k) = (I-K*H)*P

    }

    /**
     * @brief 获取指定索引的状态估计值
     *
     * @param state_index 状态向量中的索引位置（从0开始）
     * @return double 返回对应索引位置的状态估计值；如果滤波器未初始化则返回0
     *
     * 该函数用于提取状态向量中特定维度的估计值。例如，在位置-速度模型中，
     * state_index=0可能对应位置，state_index=1可能对应速度。
     * 调用前需确保滤波器已通过setup函数完成初始化。
     */
    double kalman_filter::output(int state_index)
    {
        if(this->is_initialized)  // 检查滤波器是否已初始化
        {
            return this->states(state_index, 0);  // 返回状态向量中指定索引的估计值
        }
        else
        {
            return 0;  // 滤波器未初始化时返回0
        }
    }

}  // namespace onboardDetector
