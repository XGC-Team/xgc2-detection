/*
	FILE: fakeDetector.h
	---------------------
	fake dynamic obtacle detector for gazebo simulation
*/
#ifndef FAKEDETECTOR_H
#define FAKEDETECTOR_H
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <onboard_detector_lv/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <onboard_detector_lv/GetDynamicObstacles.h>
#include <thread>
#include <mutex>
#include <deque>

using std::cout; using std::endl;

namespace onboardDetector{
	/**
	 * @class fakeDetector
	 * @brief 用于Gazebo仿真的虚拟动态障碍物检测器类
	 *
	 * 该类通过订阅Gazebo的模型状态信息来模拟真实的动态障碍物检测器
	 * 能够获取指定障碍物的位置、速度、尺寸等信息，并提供传感器范围内的障碍物检测功能
	 * 同时支持障碍物历史轨迹记录和可视化
	 */
	class fakeDetector{
	private:
		// ROS通信相关成员
		ros::NodeHandle nh_;                      // ROS节点句柄
		ros::Timer obstaclePubTimer_;             // 障碍物发布定时器
		ros::Timer visTimer_;                     // 可视化发布定时器
		ros::Timer histTimer_;                    // 历史轨迹记录定时器
		ros::Subscriber gazeboSub_;               // Gazebo模型状态订阅器
		ros::Publisher visPub_;                   // 边界框可视化发布器
		ros::Publisher historyTrajPub_;           // 障碍物历史轨迹发布器
		ros::Subscriber odomSub_;                 // 机器人里程计订阅器
		ros::ServiceServer getDynamicObstacleServer_;  // 获取动态障碍物服务服务器

		// 障碍物跟踪相关成员
		int histSize_;                            // 历史轨迹队列大小
		std::vector<std::string> targetObstacle_; // 目标障碍物名称列表
		std::vector<int> targetIndex_;            // 目标障碍物在Gazebo模型列表中的索引
		bool firstTime_;                          // 首次运行标志
		std::vector<onboardDetector::box3D> obstacleMsg_;      // 当前障碍物信息列表
		std::vector<onboardDetector::box3D> lastObVec_;        // 上一时刻障碍物信息列表
		std::vector<ros::Time> lastTimeVec_;                   // 上一时刻时间戳列表
		std::vector<std::vector<double>> lastTimeVel_;         // 上一时刻速度列表（用于速度估计）
		std::vector<std::deque<onboardDetector::box3D>> obstacleHist_;  // 障碍物历史轨迹队列

		// 可视化相关成员
		nav_msgs::Odometry odom_;                 // 机器人当前里程计信息
		double colorDistance_;                    // 可视化颜色渐变距离阈值
		visualization_msgs::MarkerArray visMsg_;  // 可视化消息数组

	public:
		/**
		 * @brief 构造函数，初始化虚拟检测器
		 * @param nh ROS节点句柄，用于创建订阅器、发布器和服务
		 */
		fakeDetector(const ros::NodeHandle& nh);

		/**
		 * @brief 获取动态障碍物服务回调函数
		 * @param req 服务请求，包含查询参数（如传感器视场角等）
		 * @param res 服务响应，包含检测到的动态障碍物信息
		 * @return 服务是否成功执行
		 */
		bool getDynamicObstacles(onboard_detector_lv::GetDynamicObstacles::Request& req,
								 onboard_detector_lv::GetDynamicObstacles::Response& res);

		/**
		 * @brief 可视化定时器回调函数，定期发布障碍物可视化信息
		 * @param 定时器事件（未使用）
		 */
		void visCB(const ros::TimerEvent&);

		/**
		 * @brief Gazebo模型状态订阅回调函数，接收并处理所有模型的状态信息
		 * @param allStates 包含所有Gazebo模型状态的消息指针
		 */
		void stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates);

		/**
		 * @brief 里程计订阅回调函数，更新机器人当前位置和姿态信息
		 * @param odom 里程计消息指针，包含机器人的位姿和速度信息
		 */
		void odomCB(const nav_msgs::OdometryConstPtr& odom);

		/**
		 * @brief 历史轨迹记录定时器回调函数，定期保存障碍物历史状态
		 * @param 定时器事件（未使用）
		 */
		void histCB(const ros::TimerEvent&);

		/**
		 * @brief 在Gazebo模型列表中查找目标障碍物的索引
		 * @param modelNames Gazebo中所有模型的名称列表
		 * @return 目标障碍物在模型列表中的索引向量引用
		 */
		std::vector<int>& findTargetIndex(const std::vector<std::string>& modelNames);

		/**
		 * @brief 更新可视化消息，生成障碍物边界框的Marker消息
		 */
		void updateVisMsg();

		/**
		 * @brief 发布障碍物信息（目前未使用）
		 */
		void publishObstacles();

		/**
		 * @brief 发布障碍物可视化消息到RViz
		 */
		void publishVisualization();

		/**
		 * @brief 发布障碍物历史轨迹可视化消息
		 */
		void publishHistoryTraj();

		/**
		 * @brief 判断障碍物是否在传感器视场范围内
		 * @param ob 障碍物3D边界框信息
		 * @param fov 传感器视场角（弧度）
		 * @return 如果障碍物在视场范围内返回true，否则返回false
		 */
		bool isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov);

		/**
		 * @brief 获取所有检测到的障碍物信息
		 * @param obstacles 输出参数，存储障碍物边界框信息的向量
		 * @param robotSize 机器人尺寸（可选），用于膨胀障碍物边界框，默认为零向量
		 */
		void getObstacles(std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));

		/**
		 * @brief 获取传感器视场范围内的障碍物信息
		 * @param fov 传感器视场角（弧度）
		 * @param obstacles 输出参数，存储视场范围内障碍物边界框信息的向量
		 * @param robotSize 机器人尺寸（可选），用于膨胀障碍物边界框，默认为零向量
		 */
		void getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));

		/**
		 * @brief 获取动态障碍物的历史轨迹信息
		 * @param posHist 输出参数，存储每个障碍物的历史位置序列
		 * @param velHist 输出参数，存储每个障碍物的历史速度序列
		 * @param sizeHist 输出参数，存储每个障碍物的历史尺寸序列
		 * @param robotSize 机器人尺寸（可选），用于膨胀障碍物边界框，默认为零向量
		 */
		void getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, std::vector<std::vector<Eigen::Vector3d>>& velHist, std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));
	};
}

#endif
