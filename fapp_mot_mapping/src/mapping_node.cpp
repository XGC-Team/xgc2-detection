// ROS核心库头文件
#include "ros/ros.h"
// 映射管理器头文件，包含MappingRos类定义
#include "mapping_manager.h"

// 使用mot_mapping命名空间
using namespace mot_mapping;

/**
 * @brief 映射节点主函数
 *
 * 该节点是FAPP(Fast and Adaptive Path Planning)系统中的运动对象映射模块的入口点。
 * 主要功能：
 * - 初始化ROS节点
 * - 创建并配置映射管理器
 * - 处理运动对象的环境映射和跟踪
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数值数组
 * @return int 程序退出状态码，0表示正常退出
 */
int main(int argc, char** argv) {
  // 初始化ROS节点，节点名称为"tracker_node"
  // 该节点负责跟踪和映射环境中的运动对象
  ros::init(argc, argv, "tracker_node");

  // 创建私有节点句柄，使用"~"前缀访问私有命名空间下的参数
  // 这样可以方便地从launch文件或参数服务器加载节点特定的配置
  ros::NodeHandle nh("~");

  // 创建映射ROS接口对象
  // MappingRos类负责处理与ROS系统的交互，包括订阅传感器数据、发布地图信息等
  MappingRos mapping_ros(nh);

  // 初始化映射管理器
  // 该函数会加载配置参数、订阅必要的话题、设置发布器和定时器等
  mapping_ros.init();

  // 进入ROS事件循环，持续处理回调函数
  // 节点将保持运行状态，响应订阅的消息和定时器事件，直到收到关闭信号
  ros::spin();

  // 程序正常退出
  return 0;
}