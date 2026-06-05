/*
	文件名: detector_node.cpp
	--------------------------
	功能: 运行动态障碍物检测器节点
	说明: 这是LV-DOT(Learning-based Visual Dynamic Obstacle Tracking)系统的主节点程序，
	      负责初始化并运行动态障碍物检测器，用于实时检测和跟踪环境中的运动障碍物
*/
#include <ros/ros.h>
#include <onboard_detector_lv/dynamicDetector.h>

/**
 * @brief 主函数 - 动态障碍物检测器节点入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码，0表示正常退出
 *
 * 功能描述:
 * 1. 初始化ROS节点
 * 2. 创建动态检测器对象
 * 3. 进入ROS事件循环，持续处理传感器数据和检测动态障碍物
 */
int main(int argc, char** argv){
	// 初始化ROS节点，节点名称为"dyanmic_detector_node"
	// 该节点负责订阅传感器数据(如点云、深度图像)并发布检测到的动态障碍物信息
	ros::init(argc, argv, "dyanmic_detector_node");

	// 创建节点句柄，用于ROS通信(话题订阅/发布、参数服务器访问等)
	ros::NodeHandle nh;

	// 实例化动态检测器对象
	// 该对象封装了动态障碍物检测的核心算法，包括:
	// - 点云数据处理
	// - 运动物体分割
	// - 障碍物轨迹跟踪
	// - 预测未来运动状态
	onboardDetector::dynamicDetector d (nh);

	// 进入ROS事件循环，阻塞并持续处理回调函数
	// 这使得检测器能够持续接收传感器数据并实时更新障碍物信息
	ros::spin();

	// 正常退出程序
	return 0;
}
