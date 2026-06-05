/*
	FILE: kalman_filter.h
	--------------------------------------
	header of kalman_filter velocity estimator
	卡尔曼滤波器速度估计器头文件
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

namespace onboardDetector{
    /**
     * @class kalman_filter
     * @brief 卡尔曼滤波器类，用于动态目标的速度估计
     *
     * 实现标准卡尔曼滤波器算法，包含预测和更新两个步骤。
     * 主要用于估计和跟踪运动目标的状态（位置、速度等）。
     *
     * 卡尔曼滤波器方程：
     * 预测步骤：
     *   x_pred = A * x + B * u
     *   P_pred = A * P * A^T + Q
     * 更新步骤：
     *   K = P_pred * H^T * (H * P_pred * H^T + R)^(-1)
     *   x = x_pred + K * (z - H * x_pred)
     *   P = (I - K * H) * P_pred
     */
    class kalman_filter
    {
        private:
        // 私有成员变量

        /** @brief 滤波器初始化标志，标识滤波器是否已完成初始化配置 */
        bool is_initialized;

        /** @brief 状态向量 x，存储系统的当前状态估计值（如位置、速度等） */
        MatrixXd states;

        /** @brief 状态转移矩阵 A，描述系统状态从 k-1 时刻到 k 时刻的演化规律 */
        MatrixXd A; // state matrix

        /** @brief 控制输入矩阵 B，描述控制输入 u 对状态的影响 */
        MatrixXd B; // input matrix

        /** @brief 观测矩阵 H，将状态空间映射到观测空间，描述如何从状态获得测量值 */
        MatrixXd H; // observation matrix

        /** @brief 状态估计协方差矩阵 P，表示状态估计的不确定性/误差协方差 */
        MatrixXd P; // uncertianty

        /** @brief 过程噪声协方差矩阵 Q，描述系统模型的不确定性和过程扰动 */
        MatrixXd Q; // process noise

        /** @brief 观测噪声协方差矩阵 R，描述测量过程的噪声和不确定性 */
        MatrixXd R; // obsevation noise

        public:
        /**
         * @brief 构造函数
         *
         * 创建一个未初始化的卡尔曼滤波器对象。
         * 需要调用 setup() 函数进行初始化后才能使用。
         */
        kalman_filter();

        /**
         * @brief 设置和初始化卡尔曼滤波器的所有参数
         *
         * @param states 初始状态向量 x_0，维度为 n×1，n 为状态维数
         * @param A 状态转移矩阵，维度为 n×n
         * @param B 控制输入矩阵，维度为 n×m，m 为控制输入维数
         * @param H 观测矩阵，维度为 p×n，p 为观测维数
         * @param P 初始状态协方差矩阵，维度为 n×n
         * @param Q 过程噪声协方差矩阵，维度为 n×n
         * @param R 观测噪声协方差矩阵，维度为 p×p
         *
         * @note 此函数必须在使用滤波器之前调用一次
         * @note 所有矩阵的维度必须相互兼容，否则会导致运行时错误
         */
        void setup(const MatrixXd& states,
                   const MatrixXd& A,
                   const MatrixXd& B,
                   const MatrixXd& H,
                   const MatrixXd& P,
                   const MatrixXd& Q,
                   const MatrixXd& R);

        /**
         * @brief 设置状态转移矩阵 A
         *
         * 在某些情况下，采样时间会发生变化，导致状态转移矩阵需要更新。
         * 例如，对于离散化的连续系统，A 通常依赖于采样间隔 dt。
         *
         * @param A 新的状态转移矩阵，维度必须与原矩阵相同 (n×n)
         *
         * @note 当系统采样时间不固定时，需要在每次估计前调用此函数更新 A
         */
        void setA(const MatrixXd& A);

        /**
         * @brief 执行卡尔曼滤波的预测和更新步骤
         *
         * 完整的卡尔曼滤波循环，包括：
         * 1. 预测步骤：根据系统模型预测当前状态
         * 2. 更新步骤：利用新的观测值修正预测结果
         *
         * @param z 观测向量，维度为 p×1，包含当前时刻的测量值
         * @param u 控制输入向量，维度为 m×1，包含当前时刻的控制输入
         *
         * @note 此函数会更新内部状态 states 和协方差矩阵 P
         * @note 每次获得新的观测数据时调用此函数进行状态估计
         */
        void estimate(const MatrixXd& z, const MatrixXd& u);

        /**
         * @brief 读取状态向量中指定索引的状态值
         *
         * @param state_index 状态索引，范围为 [0, n-1]，n 为状态向量维数
         *
         * @return 返回对应索引位置的状态估计值
         *
         * @note 索引从 0 开始
         * @note 调用前应确保 state_index 在有效范围内，否则可能导致访问越界
         *
         * 使用示例：
         *   double velocity = kf.output(1);  // 获取状态向量的第2个元素（索引1）
         */
        double output(int state_index);
    };
}

#endif