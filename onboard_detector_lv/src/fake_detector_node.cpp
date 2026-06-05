/*
	FILE: fake_detector_node.cpp
	--------------------------
	Run fake detector for simulation

	功能说明：
	虚拟动态障碍物检测器节点的主程序入口
	该节点用于Gazebo仿真环境中模拟真实的动态障碍物检测器
	通过订阅Gazebo的模型状态信息，实现对指定动态障碍物的跟踪和检测
	为无人机等移动机器人的动态避障规划提供障碍物信息服务
*/

#include <ros/ros.h>
#include <onboard_detector_lv/fakeDetector.h>

/**
 * @brief 虚拟检测器节点主函数
 *
 * 主要功能：
 * 1. 初始化ROS节点，节点名称为"fake_detector"
 * 2. 创建虚拟检测器对象，启动障碍物检测服务
 * 3. 进入ROS事件循环，持续处理回调函数
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码，正常退出返回0
 */
int main(int argc, char** argv){
	// 初始化ROS节点，节点名称为"fake_detector"
	ros::init(argc, argv, "fake_detector");

	// 创建ROS节点句柄，用于管理节点资源（订阅器、发布器、服务等）
	ros::NodeHandle nh;

	// 创建虚拟检测器对象，构造函数中会完成以下初始化：
	// - 订阅Gazebo模型状态话题（/gazebo/model_states）
	// - 订阅机器人里程计话题
	// - 创建动态障碍物查询服务
	// - 启动可视化和历史轨迹记录定时器
	onboardDetector::fakeDetector d (nh);

	// 进入ROS事件循环，阻塞等待并处理所有回调函数
	// 包括：订阅话题回调、定时器回调、服务请求回调等
	ros::spin();

	// 程序正常退出
	return 0;
}
