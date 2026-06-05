/*
	FILE: fakeDetector.cpp
	-----------------------
	功能：模拟检测器的函数定义
	说明：从Gazebo仿真环境中获取真实障碍物状态，用于测试和验证规划算法
*/

#include <onboard_detector_lv/fakeDetector.h>

namespace onboardDetector{
	/**
	 * @brief 模拟检测器构造函数
	 * @param nh ROS节点句柄
	 *
	 * 功能：
	 * 1. 加载ROS参数（目标障碍物类型、颜色距离、里程计话题等）
	 * 2. 初始化订阅者（Gazebo模型状态、里程计）
	 * 3. 初始化发布者（历史轨迹、可视化）
	 * 4. 设置定时器和服务
	 */
	fakeDetector::fakeDetector(const ros::NodeHandle& nh) : nh_(nh){
		// 加载ROS参数：
		// 1. 目标障碍物类型（从Gazebo中筛选的障碍物名称前缀）
		if (not this->nh_.getParam("target_obstacle", this->targetObstacle_)){
			this->targetObstacle_ = std::vector<std::string> {"person", "obstacle"};
			cout << "[Fake Detector]: No target obstacle param. Use default value." << endl;
		}

		// 2. 颜色距离阈值（用于可视化时根据距离改变障碍物颜色）
		if (not this->nh_.getParam("color_distance", this->colorDistance_)){
			this->colorDistance_ = 5.0; // 在此距离内的障碍物将以红色显示
			cout << "[Fake Detector]: No color distance param. Use default value: 5.0m." << endl;
		}

		// 重复参数读取（可能是代码冗余）
		if (not this->nh_.getParam("color_distance", this->colorDistance_)){
			this->colorDistance_ = 5.0; // 在此距离内的障碍物将以红色显示
			cout << "[Fake Detector]: No color distance param. Use default value: 5.0m." << endl;
		}

		// 3. 里程计话题名称
		std::string odomTopicName;
		if (not this->nh_.getParam("odom_topic", odomTopicName)){
			odomTopicName = "/CERLAB/quadcopter/odom";
			cout << "[Fake Detector]: No odom topic param. Use default: /CERLAB/quadcopter/odom" << endl;
		}

		// 4. 跟踪历史记录大小（用于存储障碍物的历史轨迹）
        if (not this->nh_.getParam("history_size", this->histSize_)){
            this->histSize_ = 5;
            std::cout << "[Fake Detector]: No tracking history size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << "[Fake Detector]: The history for tracking is set to: " << this->histSize_ << std::endl;
        }  


		// 初始化标志位（首次运行时需要查找目标障碍物索引）
		this->firstTime_ = true;

		// 订阅Gazebo模型状态（获取所有障碍物的真实位置、速度等信息）
		this->gazeboSub_ = this->nh_.subscribe("/gazebo/model_states", 10, &fakeDetector::stateCB, this);

		// 订阅机器人里程计（获取机器人当前位置，用于判断障碍物是否在传感器范围内）
		this->odomSub_ = this->nh_.subscribe(odomTopicName, 10, &fakeDetector::odomCB, this);
		// this->odomSub_ = this->nh_.subscribe("/mavros/local_position/odom", 10, &fakeDetector::odomCB, this);

		// 发布历史轨迹可视化
		this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/history_trajectories", 10);
		this->histTimer_ = this->nh_.createTimer(ros::Duration(0.033), &fakeDetector::histCB, this); // 30Hz更新频率

		// 发布障碍物边界框可视化
		this->visPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/GT_obstacle_bbox", 10);
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &fakeDetector::visCB, this); // 20Hz更新频率

		// 提供获取动态障碍物的服务接口
		this->getDynamicObstacleServer_ = this->nh_.advertiseService("fake_detector/getDynamicObstacles", &fakeDetector::getDynamicObstacles, this);
	}



	/**
	 * @brief ROS服务回调函数：获取动态障碍物信息
	 * @param req 请求参数（包含机器人当前位置和检测范围）
	 * @param res 响应参数（返回障碍物的位置、速度、尺寸）
	 * @return 服务调用成功返回true
	 *
	 * 功能：
	 * 1. 计算所有障碍物与机器人的距离（仅考虑水平距离）
	 * 2. 筛选出指定范围内的障碍物
	 * 3. 按距离从近到远排序
	 * 4. 返回排序后的障碍物信息
	 */
    bool fakeDetector::getDynamicObstacles(onboard_detector_lv::GetDynamicObstacles::Request& req,
                                           onboard_detector_lv::GetDynamicObstacles::Response& res) {
        // 获取机器人当前位置
		Eigen::Vector3d currPos (req.current_position.x, req.current_position.y, req.current_position.z);

        // 存储障碍物及其与机器人的距离
        std::vector<std::pair<double, onboardDetector::box3D>> obstaclesWithDistances;

        // 遍历所有障碍物，计算距离并筛选
        for (const onboardDetector::box3D& bbox : this->obstacleMsg_) {
            Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
            Eigen::Vector3d diff = currPos - obsPos;
            diff(2) = 0.; // 仅考虑水平距离（忽略高度差）
            double distance = diff.norm();
			if (distance <= req.range) { // 如果障碍物在检测范围内
                obstaclesWithDistances.push_back(std::make_pair(distance, bbox));
            }
        }

        // 按距离从近到远排序（升序）
        std::sort(obstaclesWithDistances.begin(), obstaclesWithDistances.end(),
                [](const std::pair<double, onboardDetector::box3D>& a, const std::pair<double, onboardDetector::box3D>& b) {
                    return a.first < b.first;
                });

        // 将排序后的障碍物信息填充到响应消息中
        for (const auto& item : obstaclesWithDistances){
            const onboardDetector::box3D& bbox = item.second;

            geometry_msgs::Vector3 pos;
            geometry_msgs::Vector3 vel;
            geometry_msgs::Vector3 size;

            // 填充位置信息
            pos.x = bbox.x;
            pos.y = bbox.y;
            pos.z = bbox.z;

            // 填充速度信息
            vel.x = bbox.Vx;
            vel.y = bbox.Vy;
            vel.z = bbox.Vz;

            // 填充尺寸信息
            size.x = bbox.x_width;
            size.y = bbox.y_width;
            size.z = bbox.z_width;

            res.position.push_back(pos);
            res.velocity.push_back(vel);
            res.size.push_back(size);
        }

        return true;
    }


	/**
	 * @brief 可视化定时器回调函数
	 * @param 定时器事件（未使用）
	 *
	 * 功能：定期发布历史轨迹和障碍物边界框的可视化
	 */
	void fakeDetector::visCB(const ros::TimerEvent&){
		this->publishHistoryTraj();
		this->publishVisualization();
	}

	/**
	 * @brief Gazebo模型状态回调函数
	 * @param allStates 所有Gazebo模型的状态信息
	 *
	 * 功能：
	 * 1. 首次运行时查找目标障碍物的索引
	 * 2. 提取每个障碍物的位置、速度和尺寸信息
	 * 3. 通过数值微分计算障碍物速度（防止高频噪声）
	 * 4. 从Gazebo模型名称中解析障碍物尺寸
	 */
	void fakeDetector::stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates){
		bool update = false; // 标志是否需要更新速度估计
		if (this->firstTime_){
			// 首次运行时查找目标障碍物在模型列表中的索引
			this->targetIndex_ = this->findTargetIndex(allStates->name);
			this->firstTime_ = false;
		}
		std::vector<onboardDetector::box3D> obVec;
		onboardDetector::box3D ob;
		geometry_msgs::Pose p;
		geometry_msgs::Twist tw;
		for (int i=0; i<int(this->targetIndex_.size()); ++i){
			std::string name = allStates->name[this->targetIndex_[i]];
			// 1. 获取位置和速度
			p = allStates->pose[this->targetIndex_[i]];
			tw = allStates->twist[this->targetIndex_[i]];
			ob.x = p.position.x;
			ob.y = p.position.y;
			// 对于"person"类型的障碍物，中心点向上偏移0.9m（模拟人体中心）
			if (name.size() >= 6 and name.compare(0, 6, "person") == 0){
				ob.z = p.position.z + 0.9;
			}
			else{
				ob.z = p.position.z;
			}

			// 2. 计算速度（通过数值微分）
			if (this->lastObVec_.size() == 0){
				// 首次运行，速度初始化为0
				ob.Vx = 0.0;
				ob.Vy = 0.0;
				ob.Vz = 0.0;
				ros::Time lastTime = ros::Time::now();
				this->lastTimeVec_.push_back(lastTime);
				this->lastTimeVel_.push_back(std::vector<double> {0, 0, 0});
				update = true;
			}
			else{
				// 通过位置差分计算速度
				ros::Time currTime = ros::Time::now();
				double dT = (currTime.toSec() - this->lastTimeVec_[i].toSec());
				if (dT >= 0.1){ // 每0.1秒更新一次速度估计（降低高频噪声）
					double vx = (ob.x - this->lastObVec_[i].x)/dT;
					double vy = (ob.y - this->lastObVec_[i].y)/dT;
					double vz = (ob.z - this->lastObVec_[i].z)/dT;
					ob.Vx = vx;
					ob.Vy = vy;
					ob.Vz = vz;
					this->lastTimeVel_[i][0] = vx;
					this->lastTimeVel_[i][1] = vy;
					this->lastTimeVel_[i][2] = vz;
					this->lastTimeVec_[i] = ros::Time::now();
					update = true;
				}
				else{
					// 时间间隔太短，使用上次估计的速度
					ob.Vx = this->lastTimeVel_[i][0];
					ob.Vy = this->lastTimeVel_[i][1];
					ob.Vz = this->lastTimeVel_[i][2];
				}
			}
			// 3. 从Gazebo模型名称中解析尺寸信息
			// Gazebo模型名称格式：name_xxx_yyy_zzz（最后9个字符编码了xyz尺寸）
			double xsize, ysize, zsize;
			int xsizeStartIdx = name.size() - 1 - 1 - 3 * 3; // x尺寸起始索引
			std::string xsizeStr = name.substr(xsizeStartIdx, 3);
			xsize = std::stod(xsizeStr);

			int ysizeStartIdx = name.size() - 1 - 3 * 2; // y尺寸起始索引
			std::string ysizeStr = name.substr(ysizeStartIdx, 3);
			ysize = std::stod(ysizeStr);

			int zsizeStartIdx = name.size() - 3; // z尺寸起始索引
			std::string zsizeStr = name.substr(zsizeStartIdx, 3);
			zsize = std::stod(zsizeStr);

			ob.x_width = xsize;
			ob.y_width = ysize;
			ob.z_width = zsize;
			obVec.push_back(ob);
		}
		if (update){
			// 更新上一次的障碍物状态（用于下次速度计算）
			this->lastObVec_ = obVec;
		}
		// 更新当前障碍物信息
		this->obstacleMsg_ = obVec;
		// ros::Rate r (60);
		// r.sleep();
	}

	/**
	 * @brief 里程计回调函数
	 * @param odom 机器人里程计消息
	 *
	 * 功能：保存最新的机器人位姿，用于判断障碍物是否在传感器范围内
	 */
	void fakeDetector::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
	}

	/**
	 * @brief 历史记录定时器回调函数
	 * @param 定时器事件（未使用）
	 *
	 * 功能：
	 * 1. 维护每个障碍物的历史轨迹队列（FIFO）
	 * 2. 限制历史记录长度不超过histSize_
	 * 3. 用于轨迹预测和可视化
	 */
	void fakeDetector::histCB(const ros::TimerEvent&){
		if (this->obstacleHist_.size() == 0){
			// 首次运行，根据障碍物数量初始化历史记录容器
			this->obstacleHist_.resize(this->obstacleMsg_.size());
		}
		for (int i=0; i<int(this->obstacleMsg_.size());i++){
			if (int(this->obstacleHist_[i].size()) >= this->histSize_){
				// 如果历史记录已满，删除最旧的记录
				this->obstacleHist_[i].pop_back();
			}
			// 将当前状态添加到历史记录队首
			this->obstacleHist_[i].push_front(this->obstacleMsg_[i]);
		}
	}

	/**
	 * @brief 查找目标障碍物在Gazebo模型列表中的索引
	 * @param modelNames Gazebo中所有模型的名称列表
	 * @return 目标障碍物的索引列表
	 *
	 * 功能：
	 * 根据配置的目标障碍物名称前缀（如"person"、"obstacle"），
	 * 在Gazebo模型列表中查找匹配的模型索引，用于后续提取障碍物状态
	 */
	std::vector<int>& fakeDetector::findTargetIndex(const std::vector<std::string>& modelNames){
		static std::vector<int> targetIndex;
		int countID = 0;
		for (std::string name : modelNames){
			// 检查模型名称是否以目标前缀开头
			for (std::string targetName : this->targetObstacle_){
				if (name.compare(0, targetName.size(), targetName) == 0){
					targetIndex.push_back(countID);
				}
			}
			++countID;
		}
		return targetIndex;
	}

	/**
	 * @brief 更新可视化消息
	 *
	 * 功能：
	 * 1. 为每个障碍物生成3D边界框（由12条线段组成）
	 * 2. 根据障碍物是否在传感器范围内设置不同颜色（红色/绿色）
	 * 3. 准备RViz可视化消息
	 */
	void fakeDetector::updateVisMsg(){
		std::vector<visualization_msgs::Marker> bboxVec;
		int obIdx = 0;
		for (const onboardDetector:: box3D& obstacle : this->obstacleMsg_){

			// 为每个障碍物生成12条边的3D包围盒
			geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
			// 上层四个顶点（z正方向）
			p1.x = obstacle.x+obstacle.x_width/2; p1.y = obstacle.y+obstacle.y_width/2; p1.z = obstacle.z+obstacle.z_width/2;
			p2.x = obstacle.x-obstacle.x_width/2; p2.y = obstacle.y+obstacle.y_width/2; p2.z = obstacle.z+obstacle.z_width/2;
			p3.x = obstacle.x+obstacle.x_width/2; p3.y = obstacle.y-obstacle.y_width/2; p3.z = obstacle.z+obstacle.z_width/2;
			p4.x = obstacle.x-obstacle.x_width/2; p4.y = obstacle.y-obstacle.y_width/2; p4.z = obstacle.z+obstacle.z_width/2;

			// 下层四个顶点（z负方向）
			p5.x = obstacle.x+obstacle.x_width/2; p5.y = obstacle.y+obstacle.y_width/2; p5.z = obstacle.z-obstacle.z_width/2;
			p6.x = obstacle.x-obstacle.x_width/2; p6.y = obstacle.y+obstacle.y_width/2; p6.z = obstacle.z-obstacle.z_width/2;
			p7.x = obstacle.x+obstacle.x_width/2; p7.y = obstacle.y-obstacle.y_width/2; p7.z = obstacle.z-obstacle.z_width/2;
			p8.x = obstacle.x-obstacle.x_width/2; p8.y = obstacle.y-obstacle.y_width/2; p8.z = obstacle.z-obstacle.z_width/2;

			// 定义12条边（连接8个顶点形成立方体框架）
			std::vector<geometry_msgs::Point> line1Vec {p1, p2};   // 上层边1
			std::vector<geometry_msgs::Point> line2Vec {p1, p3};   // 上层边2
			std::vector<geometry_msgs::Point> line3Vec {p2, p4};   // 上层边3
			std::vector<geometry_msgs::Point> line4Vec {p3, p4};   // 上层边4
			std::vector<geometry_msgs::Point> line5Vec {p1, p5};   // 竖直边1
			std::vector<geometry_msgs::Point> line6Vec {p2, p6};   // 竖直边2
			std::vector<geometry_msgs::Point> line7Vec {p3, p7};   // 竖直边3
			std::vector<geometry_msgs::Point> line8Vec {p4, p8};   // 竖直边4
			std::vector<geometry_msgs::Point> line9Vec {p5, p6};   // 下层边1
			std::vector<geometry_msgs::Point> line10Vec {p5, p7};  // 下层边2
			std::vector<geometry_msgs::Point> line11Vec {p6, p8};  // 下层边3
			std::vector<geometry_msgs::Point> line12Vec {p7, p8};  // 下层边4

			std::vector<std::vector<geometry_msgs::Point>> allLines{
				line1Vec,
				line2Vec,
				line3Vec,
				line4Vec,
				line5Vec,
				line6Vec,
				line7Vec,
				line8Vec,
				line9Vec,
				line10Vec,
				line11Vec,
				line12Vec
			};

			int count = 0;
			std::string name = "GT osbtacles" + std::to_string(obIdx);
			// 为每条边创建一个Marker
			for (std::vector<geometry_msgs::Point> lineVec: allLines){
				visualization_msgs::Marker line;

				line.header.frame_id = "map";
				line.ns = name;
				line.points = lineVec;
				line.id = count;
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.lifetime = ros::Duration(0.5); // 可视化持续时间0.5秒
				line.scale.x = 0.05; // 线宽
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.a = 1.0; // 不透明度
				// 根据障碍物是否在传感器范围内设置颜色
				if (this->isObstacleInSensorRange(obstacle, PI_const)){
					line.color.r = 1; // 红色：在传感器范围内
					line.color.g = 0;
					line.color.b = 0;
				}
				else{
					line.color.r = 0; // 绿色：不在传感器范围内
					line.color.g = 1;
					line.color.b = 0;
				}
				++count;
				bboxVec.push_back(line);
			}
			++obIdx;
		}
		this->visMsg_.markers = bboxVec;
	}

	/**
	 * @brief 发布障碍物历史轨迹可视化
	 *
	 * 功能：
	 * 1. 遍历所有障碍物的历史记录
	 * 2. 仅发布在传感器范围内的障碍物轨迹
	 * 3. 使用LINE_STRIP类型在RViz中绘制轨迹线
	 */
	void fakeDetector::publishHistoryTraj(){
		if (this->obstacleHist_.size() != 0){
			visualization_msgs::MarkerArray trajMsg;
			int countMarker = 0;
			for (size_t i=0; i<this->obstacleHist_.size(); ++i){
				// 仅可视化在传感器范围内的障碍物历史轨迹（FOV=2π，即全方位）
				if (this->isObstacleInSensorRange(this->obstacleHist_[i][0],2*M_PI)){
					visualization_msgs::Marker traj;
					traj.header.frame_id = "map";
					traj.header.stamp = ros::Time::now();
					traj.ns = "fake_detector";
					traj.id = countMarker;
					traj.type = visualization_msgs::Marker::LINE_STRIP; // 连续线段
					traj.scale.x = 0.1; // 线宽
					traj.scale.y = 0.1;
					traj.scale.z = 0.1;
					traj.color.a = 1.0;
					traj.color.r = 0.0;
					traj.color.g = 1.0; // 绿色轨迹
					traj.color.b = 0.0;
					// 添加历史轨迹点（从最新到最旧）
					for (size_t j=0; j<this->obstacleHist_[i].size(); ++j){
						geometry_msgs::Point p1;
						onboardDetector::box3D box1 = this->obstacleHist_[i][j];
						p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
						traj.points.push_back(p1);
					}

					++countMarker;
					trajMsg.markers.push_back(traj);
				}
			}
			this->historyTrajPub_.publish(trajMsg);
		}
	}


	/**
	 * @brief 发布障碍物边界框可视化
	 *
	 * 功能：更新并发布所有障碍物的3D边界框到RViz
	 */
	void fakeDetector::publishVisualization(){
		this->updateVisMsg();
		this->visPub_.publish(this->visMsg_);
	}

	/**
	 * @brief 判断障碍物是否在传感器范围内
	 * @param ob 障碍物信息
	 * @param fov 传感器视场角（弧度）
	 * @return 如果障碍物在范围内返回true，否则返回false
	 *
	 * 功能：
	 * 1. 计算障碍物与机器人的水平距离
	 * 2. 计算障碍物相对于机器人朝向的角度
	 * 3. 判断是否同时满足距离和角度约束
	 */
	bool fakeDetector::isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov){
		Eigen::Vector3d pRobot (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d pObstacle (ob.x, ob.y, ob.z);

		Eigen::Vector3d diff = pObstacle - pRobot; // 障碍物相对机器人的位置向量
		diff(2) = 0.0; // 忽略高度差，仅考虑水平距离
		double distance = diff.norm(); // 水平距离
		double yaw = rpy_from_quaternion(this->odom_.pose.pose.orientation); // 机器人朝向（yaw角）
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0); // 机器人朝向单位向量

		double angle = angleBetweenVectors(direction, diff); // 计算夹角
		// 同时满足角度约束和距离约束
		if (angle <= fov/2 and distance <= this->colorDistance_){
			return true;
		}
		else{
			return false;
		}

	}

	/**
	 * @brief 获取所有障碍物信息（膨胀后）
	 * @param obstacles 输出参数：障碍物列表
	 * @param robotSize 机器人尺寸（用于膨胀障碍物）
	 *
	 * 功能：
	 * 1. 获取所有检测到的障碍物
	 * 2. 将障碍物尺寸膨胀（加上机器人尺寸），用于碰撞检测
	 */
	void fakeDetector::getObstacles(std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize){
		obstacles.clear();
		for (onboardDetector::box3D ob : this->obstacleMsg_){
			// 障碍物膨胀：将机器人尺寸加到障碍物尺寸上，等效于将机器人视为质点
			ob.x_width += robotSize(0);
			ob.y_width += robotSize(1);
			ob.z_width += robotSize(2);
			obstacles.push_back(ob);
		}
		// obstacles = this->obstacleMsg_;
	}

	/**
	 * @brief 获取传感器范围内的障碍物信息（膨胀后）
	 * @param fov 传感器视场角（弧度）
	 * @param obstacles 输出参数：障碍物列表
	 * @param robotSize 机器人尺寸（用于膨胀障碍物）
	 *
	 * 功能：
	 * 1. 筛选出在传感器视场范围内的障碍物
	 * 2. 将障碍物尺寸膨胀（加上机器人尺寸），用于碰撞检测
	 */
	void fakeDetector::getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize){
		obstacles.clear();
		for (onboardDetector::box3D obstacle : this->obstacleMsg_){
			if (this->isObstacleInSensorRange(obstacle, fov)){
				// 障碍物膨胀：将机器人尺寸加到障碍物尺寸上
				obstacle.x_width += robotSize(0);
				obstacle.y_width += robotSize(1);
				obstacle.z_width += robotSize(2);
				obstacles.push_back(obstacle);
			}
		}
	}

	/**
	 * @brief 获取动态障碍物的历史轨迹信息
	 * @param posHist 输出参数：所有障碍物的位置历史
	 * @param velHist 输出参数：所有障碍物的速度历史
	 * @param sizeHist 输出参数：所有障碍物的尺寸历史（膨胀后）
	 * @param robotSize 机器人尺寸（用于膨胀障碍物）
	 *
	 * 功能：
	 * 1. 提取所有在传感器范围内的障碍物历史轨迹
	 * 2. 分别返回位置、速度、尺寸的时间序列
	 * 3. 用于轨迹预测和避障规划
	 *
	 * 注意：
	 * - 仅返回当前在传感器范围内的障碍物（FOV=2π，即全方位）
	 * - 速度的z分量设为0（假设障碍物在水平面运动）
	 * - 障碍物尺寸已膨胀（加上机器人尺寸）
	 */
	void fakeDetector::getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, std::vector<std::vector<Eigen::Vector3d>>& velHist, std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize){
		posHist.clear();
        velHist.clear();
        sizeHist.clear();

        if (this->obstacleHist_.size()){
            for (size_t i=0 ; i<this->obstacleHist_.size() ; ++i){
				// 仅返回在传感器范围内的障碍物历史（FOV=2π，全方位检测）
				if (this->isObstacleInSensorRange(this->obstacleHist_[i][0],2*M_PI)){
					std::vector<Eigen::Vector3d> obPosHist, obVelHist, obSizeHist;
					// 遍历该障碍物的所有历史记录
					for (size_t j=0; j<this->obstacleHist_[i].size() ; ++j){
						Eigen::Vector3d pos(this->obstacleHist_[i][j].x, this->obstacleHist_[i][j].y, this->obstacleHist_[i][j].z);
						Eigen::Vector3d vel(this->obstacleHist_[i][j].Vx, this->obstacleHist_[i][j].Vy, 0); // z方向速度设为0
						Eigen::Vector3d size(this->obstacleHist_[i][j].x_width, this->obstacleHist_[i][j].y_width, this->obstacleHist_[i][j].z_width);
						size += robotSize; // 障碍物膨胀
						obPosHist.push_back(pos);
						obVelHist.push_back(vel);
						obSizeHist.push_back(size);
					}
					posHist.push_back(obPosHist);
					velHist.push_back(obVelHist);
					sizeHist.push_back(obSizeHist);
				}
            }
        }
	}
}
