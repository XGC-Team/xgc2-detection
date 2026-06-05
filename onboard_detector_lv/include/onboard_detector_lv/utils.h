 /*
 	FILE: utils.h
 	--------------------------
 	机载检测器的工具函数集合
 	提供3D边界框数据结构、四元数转换、几何计算等功能
 */

#ifndef ONBOARD_DETECTOR_UTILS_H
#define ONBOARD_DETECTOR_UTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Eigen>

namespace onboardDetector{
    // 圆周率常量定义
    const double PI_const = 3.1415926;

    /**
     * @brief 3D边界框结构体
     * 用于表示三维空间中的物体边界框，包含位置、尺寸、速度、加速度及动态属性标志
     */
    struct box3D
    {
        // ========== 位置信息 ==========
        double x, y, z;                    // 边界框中心的3D坐标 (米)

        // ========== 尺寸信息 ==========
        double x_width, y_width, z_width;  // 边界框在x、y、z方向的宽度/尺寸 (米)

        // ========== 标识信息 ==========
        double id;                         // 边界框的唯一标识符

        // ========== 运动信息 ==========
        double Vx=0, Vy=0, Vz=0;          // 边界框在x、y、z方向的速度分量 (米/秒)
        double Ax=0, Ay=0, Az=0;          // 边界框在x、y、z方向的加速度分量 (米/秒²)

        // ========== 动态属性标志 ==========
        bool is_human=false;               // 是否被YOLO检测为人类（动态物体）
                                          // false: 未被YOLO检测为动态物体
                                          // true: 被YOLO检测为动态物体

        bool is_dynamic=false;            // 是否被检测为动态物体（通过YOLO或分类回调函数）
                                          // false: 未被检测为动态物体
                                          // true: 被检测为动态物体

        bool fix_size=false;              // 尺寸固定标志，用于强制未来的边界框保持固定尺寸
                                          // false: 尺寸可变
                                          // true: 尺寸固定

        bool is_dynamic_candidate=false;  // 是否为动态物体候选
                                          // 用于在正式判定为动态物体前的中间状态

        bool is_estimated=false;          // 是否为估计得到的边界框
                                          // false: 直接检测得到
                                          // true: 通过估计/预测得到
    };

    /**
     * @brief 从欧拉角(Roll-Pitch-Yaw)转换为四元数
     *
     * @param roll  滚转角 (弧度)
     * @param pitch 俯仰角 (弧度)
     * @param yaw   偏航角 (弧度)
     * @return geometry_msgs::Quaternion 转换后的四元数表示
     *
     * @note 如果yaw角度大于π，会自动归一化到[-π, π]范围内
     */
    inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        if (yaw > PI_const){
            yaw = yaw - 2*PI_const;
        }
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    /**
     * @brief 从四元数提取偏航角(Yaw)
     *
     * @param quat 输入的四元数 (geometry_msgs::Quaternion格式)
     * @return double 提取的偏航角，范围为[0, 2π] (弧度)
     *
     * @note 此函数仅返回yaw角度，roll和pitch被计算但不返回
     */
    inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
        // 返回值范围: [0, 2π]
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    /**
     * @brief 从四元数提取完整的欧拉角(Roll-Pitch-Yaw)
     *
     * @param quat  输入的四元数 (geometry_msgs::Quaternion格式)
     * @param roll  输出参数: 滚转角 (弧度)
     * @param pitch 输出参数: 俯仰角 (弧度)
     * @param yaw   输出参数: 偏航角 (弧度)
     *
     * @note 此函数通过引用参数返回所有三个欧拉角
     */
    inline void rpy_from_quaternion(const geometry_msgs::Quaternion& quat, double &roll, double &pitch, double &yaw){
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    /**
     * @brief 计算两个三维向量之间的夹角
     *
     * @param a 第一个三维向量
     * @param b 第二个三维向量
     * @return double 两向量之间的夹角 (弧度)，范围为[0, π]
     *
     * @note 使用atan2(||a×b||, a·b)公式计算，确保结果在正确范围内
     *       - a×b的模长表示两向量张成的平行四边形面积
     *       - a·b表示两向量的点积
     */
    inline double angleBetweenVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b){
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }

    /**
     * @brief 计算三维点集的几何中心(质心)
     *
     * @param points 三维点的向量集合
     * @return Eigen::Vector3d 点集的几何中心坐标
     *
     * @note 如果输入点集为空，返回零向量(0, 0, 0)
     *       几何中心计算公式: center = (Σp_i) / n，其中n为点的数量
     */
    inline Eigen::Vector3d computeCenter(const std::vector<Eigen::Vector3d> &points) {
        Eigen::Vector3d center(0.0, 0.0, 0.0);
        if (points.empty()) {
            return center;
        }

        for (const auto &p : points) {
            center += p;
        }
        center /= static_cast<double>(points.size());

        return center;
    }

    /**
     * @brief 计算三维点集相对于给定中心的标准差
     *
     * @param points 三维点的向量集合
     * @param center 参考中心点（通常是点集的几何中心）
     * @return Eigen::Vector3d 在x、y、z三个方向上的标准差
     *
     * @note 如果输入点集为空，返回零向量(0, 0, 0)
     *       标准差计算公式: σ = sqrt(Σ(p_i - center)² / n)
     *       对x、y、z三个维度分别独立计算标准差
     */
    inline Eigen::Vector3d computeStd(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &center) {
        Eigen::Vector3d stds(0.0, 0.0, 0.0);
        if (points.empty()) {
            return stds;
        }

        // 累加每个点与中心的平方距离
        for (const auto &p : points) {
            Eigen::Vector3d diff = p - center;
            stds(0) += diff(0) * diff(0);
            stds(1) += diff(1) * diff(1);
            stds(2) += diff(2) * diff(2);
        }
        // 计算方差（平方距离的平均值）
        stds /= static_cast<double>(points.size());
        // 取平方根得到标准差
        stds = stds.array().sqrt();

        return stds;
    }
}

#endif
