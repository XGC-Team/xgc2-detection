/*
 * target_ekf.hpp
 *
 * 功能：基于扩展卡尔曼滤波器(EKF)的目标跟踪器头文件
 * 用于跟踪和预测动态目标的位置和速度状态
 */

#pragma once
#include <ros/ros.h>
#include <queue>
#include <Eigen/Geometry>

/**
 * @struct Ekf
 * @brief 扩展卡尔曼滤波器(Extended Kalman Filter)结构体
 *
 * 用于跟踪动态目标的6维状态(位置和速度)。
 * 状态向量: x = [px, py, pz, vx, vy, vz]^T
 * - 前3维表示位置(position)
 * - 后3维表示速度(velocity)
 */
struct Ekf {
  typedef std::shared_ptr<Ekf> Ptr;  // 智能指针类型定义

  // ===== 基本属性 =====
  int id;                           // 目标唯一标识符
  double dt;                        // 时间步长(秒)
  ros::Time last_update_stamp_;     // 上次更新的时间戳
  int age;                          // 目标存活时长(自创建以来经过的时间步数)
  int update_num;                   // 累计更新次数，用于判断跟踪稳定性

  // ===== 卡尔曼滤波器矩阵 =====
  Eigen::MatrixXd A;                // 状态转移矩阵 (6x6): x_{k+1} = A * x_k
  Eigen::MatrixXd B;                // 控制输入矩阵 (6x6): 用于加速度控制输入
  Eigen::MatrixXd C;                // 观测矩阵 (6x6): z_k = C * x_k，全观测系统
  Eigen::MatrixXd Qt;               // 过程噪声协方差矩阵 (6x6): 系统模型的不确定性
  Eigen::MatrixXd Rt;               // 测量噪声协方差矩阵 (6x6): 传感器测量的不确定性
  Eigen::MatrixXd Sigma;            // 状态估计协方差矩阵 (6x6): 反映状态估计的不确定性
  Eigen::MatrixXd K;                // 卡尔曼增益矩阵 (6x6): 平衡预测值和测量值的权重
  Eigen::VectorXd x;                // 状态向量 (6x1): [px, py, pz, vx, vy, vz]^T

  // ===== 数据记录 =====
  std::deque<Eigen::MatrixXd> InnoCov_list;  // 新息协方差列表，用于自适应滤波

  /**
   * @brief 构造函数：初始化EKF滤波器的所有矩阵和参数
   * @param _dt 时间步长(秒)
   *
   * 初始化过程：
   * 1. 设置状态转移矩阵A（恒速运动模型）
   * 2. 设置控制输入矩阵B（用于加速度输入）
   * 3. 设置观测矩阵C（全状态观测）
   * 4. 初始化过程噪声和测量噪声协方差矩阵
   */
  Ekf(double _dt) : dt(_dt) {
    // 初始化状态转移矩阵A为单位矩阵
    A.setIdentity(6, 6);
    // 初始化状态协方差矩阵为零矩阵
    Sigma.setZero(6, 6);
    // 初始化控制输入矩阵B为零矩阵
    B.setZero(6, 6);
    // 初始化观测矩阵C为零矩阵
    C.setZero(6, 6);

    // 设置状态转移矩阵A: 恒速运动模型
    // p_{k+1} = p_k + v_k * dt
    A(0, 3) = dt;  // x位置受x速度影响
    A(1, 4) = dt;  // y位置受y速度影响
    A(2, 5) = dt;  // z位置受z速度影响

    // 设置控制输入矩阵B: 用于加速度控制
    // p_{k+1} = p_k + v_k * dt + 0.5 * a_k * dt^2
    // v_{k+1} = v_k + a_k * dt
    double t2 = dt * dt / 2;  // dt^2 / 2，用于位置更新
    B(0, 0) = t2;  // x位置受x加速度影响
    B(1, 1) = t2;  // y位置受y加速度影响
    B(2, 2) = t2;  // z位置受z加速度影响
    B(3, 3) = dt;  // x速度受x加速度影响
    B(4, 4) = dt;  // y速度受y加速度影响
    B(5, 5) = dt;  // z速度受z加速度影响

    // 设置观测矩阵C: 全状态可观测（单位矩阵）
    // 假设可以直接观测位置和速度的所有6个维度
    C(0, 0) = 1;  // 观测x位置
    C(1, 1) = 1;  // 观测y位置
    C(2, 2) = 1;  // 观测z位置
    C(3, 3) = 1;  // 观测x速度
    C(4, 4) = 1;  // 观测y速度
    C(5, 5) = 1;  // 观测z速度

    // 初始化卡尔曼增益为观测矩阵
    K = C;

    // 初始化过程噪声协方差矩阵Qt（系统模型不确定性）
    Qt.setIdentity(6, 6);
    Qt(0, 0) = 0.1;  // x位置过程噪声方差
    Qt(1, 1) = 0.1;  // y位置过程噪声方差
    Qt(2, 2) = 0.1;  // z位置过程噪声方差
    Qt(3, 3) = 0.1;  // x速度过程噪声方差
    Qt(4, 4) = 0.1;  // y速度过程噪声方差
    Qt(5, 5) = 0.1;  // z速度过程噪声方差

    // 初始化测量噪声协方差矩阵Rt（传感器测量不确定性）
    Rt.setIdentity(6, 6);
    Rt(0, 0) = 0.09;  // x位置测量噪声方差（更小，表示位置测量更准确）
    Rt(1, 1) = 0.09;  // y位置测量噪声方差
    Rt(2, 2) = 0.09;  // z位置测量噪声方差
    Rt(3, 3) = 0.4;   // x速度测量噪声方差（更大，表示速度测量不太准确）
    Rt(4, 4) = 0.4;   // y速度测量噪声方差
    Rt(5, 5) = 0.4;   // z速度测量噪声方差

    // 初始化状态向量为零
    x.setZero(6);
  }
  /**
   * @brief 预测步骤：根据运动模型预测下一时刻的状态
   *
   * EKF预测方程：
   * 1. 状态预测: x_{k|k-1} = A * x_{k-1|k-1}
   * 2. 协方差预测: P_{k|k-1} = A * P_{k-1|k-1} * A^T + Qt
   *
   * 假设恒速运动模型，不考虑外部控制输入
   */
  inline void predict() {
    // 状态预测：根据状态转移矩阵预测下一时刻状态
    x = A * x;
    // 协方差预测：更新状态估计的不确定性，加入过程噪声
    Sigma = A * Sigma * A.transpose() + Qt;
    return;
  }
  /**
   * @brief 重置滤波器：用新的观测初始化目标跟踪器
   * @param z 初始位置观测 [px, py, pz]^T
   * @param id_ 目标唯一标识符
   *
   * 用于新目标首次被检测时的初始化，或者跟踪失败后的重新初始化
   */
  inline void reset(const Eigen::Vector3d& z, int id_) {
    // 设置位置为观测值
    x.head(3) = z;
    // 速度初始化为零（假设目标初始静止或速度未知）
    x.tail(3).setZero();
    // 重置协方差矩阵为零（完全相信初始观测）
    Sigma.setZero();
    // 记录初始化时间戳
    last_update_stamp_ = ros::Time::now();
    // 重置目标年龄为1
    age = 1;
    // 重置更新计数器为0
    update_num = 0;
    // 设置目标ID
    id = id_;
  }
  /**
   * @brief 验证观测有效性：检查新的观测是否合理
   * @param z1 位置观测 [px, py, pz]^T
   * @param z2 速度观测 [vx, vy, vz]^T
   * @return true 如果观测有效（更新后的速度不超过最大值），false 否则
   *
   * 通过临时更新状态并检查速度约束来判断观测是否合理。
   * 这可以防止异常观测值污染滤波器状态。
   */
  inline bool checkValid(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    // 构建完整的观测向量 [位置, 速度]
    Eigen::VectorXd z(6);
    z << z1, z2;
    // 临时计算卡尔曼增益
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    // 临时计算更新后的状态（不实际更新）
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    // 最大速度阈值 (m/s)
    const double vmax = 4;
    // 检查更新后的速度是否超过最大值
    if (x_tmp.tail(3).norm() > vmax) {
      return false;  // 观测无效，可能是异常值
    } else {
      return true;   // 观测有效
    }
  }
  /**
   * @brief 更新步骤：用新的观测更新状态估计
   * @param z1 位置观测 [px, py, pz]^T
   * @param z2 速度观测 [vx, vy, vz]^T
   *
   * EKF更新方程：
   * 1. 新息(Innovation): y = z - C * x_{k|k-1}
   * 2. 新息协方差: S = C * P_{k|k-1} * C^T + Rt
   * 3. 卡尔曼增益: K = P_{k|k-1} * C^T * S^{-1}
   * 4. 状态更新: x_{k|k} = x_{k|k-1} + K * y
   * 5. 协方差更新: P_{k|k} = (I - K * C) * P_{k|k-1}
   */
  inline void update(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    // 构建完整的观测向量 [位置, 速度]
    Eigen::VectorXd z(6);
    z << z1, z2;
    // 计算卡尔曼增益：平衡预测值和测量值的权重
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    // 状态更新：结合预测值和观测值
    // 新息 = 观测值 - 预测观测值
    x = x + K * (z - C * x);
    // 协方差更新：减小状态估计的不确定性
    Sigma = Sigma - K * C * Sigma;

    // 更新时间戳
    last_update_stamp_ = ros::Time::now();
    // 增加更新计数
    update_num ++;
  }
  /**
   * @brief 获取当前位置估计
   * @return 位置向量 [px, py, pz]^T (米)
   */
  inline const Eigen::Vector3d pos() const {
    return x.head(3);  // 返回状态向量的前3个元素（位置）
  }

  /**
   * @brief 获取当前速度估计
   * @return 速度向量 [vx, vy, vz]^T (米/秒)
   */
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);  // 返回状态向量的后3个元素（速度）
  }
};