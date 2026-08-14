///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//
// 文件说明：
// 匈牙利算法（Hungarian Algorithm）是一种用于解决分配问题（Assignment Problem）的组合优化算法
// 主要用于在二分图中寻找最大权匹配，或者在矩阵中寻找最小代价分配
// 在运动规划中，常用于多目标跟踪、数据关联等场景，将观测与目标进行最优匹配
// 时间复杂度：O(n^3)，其中n为矩阵维度

#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <iostream>
#include <vector>

using namespace std;


/**
 * @brief 匈牙利算法类
 *
 * 该类实现了经典的匈牙利算法（Kuhn-Munkres算法），用于解决分配问题
 * 分配问题：给定一个代价矩阵，找到一种分配方案，使得总代价最小
 *
 * 应用场景：
 * - 多目标跟踪中的数据关联（将检测结果与跟踪目标匹配）
 * - 任务分配（将任务分配给工人，使总成本最小）
 * - 图像匹配（将特征点进行匹配）
 */
class HungarianAlgorithm
{
public:
	/**
	 * @brief 构造函数
	 * 初始化匈牙利算法求解器
	 */
	HungarianAlgorithm();

	/**
	 * @brief 析构函数
	 * 释放算法资源
	 */
	~HungarianAlgorithm();

	/**
	 * @brief 求解分配问题的主函数
	 *
	 * @param DistMatrix 代价矩阵（距离矩阵），DistMatrix[i][j]表示将第i个对象分配给第j个任务的代价
	 *                   矩阵可以是非方阵（行数!=列数），算法会自动处理
	 * @param Assignment 输出参数，存储分配结果。Assignment[i]=j表示第i个对象被分配给第j个任务
	 *                   如果Assignment[i]=-1，表示第i个对象未被分配
	 * @return 返回最小总代价
	 *
	 * 算法流程：
	 * 1. 行化简：每行减去该行最小值
	 * 2. 列化简：每列减去该列最小值
	 * 3. 用最少的直线覆盖所有零元素
	 * 4. 如果直线数等于矩阵维度，则找到最优解
	 * 5. 否则，调整矩阵并重复步骤3
	 */
	double Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment);

private:
	/**
	 * @brief 执行最优分配算法的核心函数
	 *
	 * @param assignment 输出参数，存储每行的分配结果
	 * @param cost 输出参数，存储最小总代价
	 * @param distMatrix 代价矩阵（一维数组形式）
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 *
	 * 该函数是匈牙利算法的核心实现，协调调用各个步骤完成求解
	 */
	void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);

	/**
	 * @brief 根据星标矩阵构建分配向量
	 *
	 * @param assignment 输出参数，分配结果向量
	 * @param starMatrix 星标矩阵，starMatrix[i*nOfColumns+j]=true表示(i,j)位置有星标
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 *
	 * 星标（star）表示已选中的分配，每行每列最多有一个星标
	 * 该函数将星标矩阵转换为更直观的分配向量形式
	 */
	void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);

	/**
	 * @brief 计算给定分配方案的总代价
	 *
	 * @param assignment 分配方案
	 * @param cost 输出参数，总代价
	 * @param distMatrix 代价矩阵
	 * @param nOfRows 矩阵行数
	 *
	 * 根据分配方案，累加对应位置的代价值
	 */
	void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);

	/**
	 * @brief 步骤2a：对每行，找到包含零的位置并进行星标标记
	 *
	 * @param assignment 分配结果
	 * @param distMatrix 代价矩阵
	 * @param starMatrix 星标矩阵，标记已选中的分配
	 * @param newStarMatrix 新星标矩阵
	 * @param primeMatrix 撇标矩阵，标记候选的分配
	 * @param coveredColumns 列覆盖标记数组
	 * @param coveredRows 行覆盖标记数组
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 * @param minDim 矩阵最小维度
	 *
	 * 遍历每行，找到值为0且所在列未被覆盖的元素，进行星标标记
	 */
	void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

	/**
	 * @brief 步骤2b：覆盖所有包含星标的列
	 *
	 * @param assignment 分配结果
	 * @param distMatrix 代价矩阵
	 * @param starMatrix 星标矩阵
	 * @param newStarMatrix 新星标矩阵
	 * @param primeMatrix 撇标矩阵
	 * @param coveredColumns 列覆盖标记数组
	 * @param coveredRows 行覆盖标记数组
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 * @param minDim 矩阵最小维度
	 *
	 * 遍历星标矩阵，将包含星标的列标记为已覆盖
	 */
	void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

	/**
	 * @brief 步骤3：覆盖所有零元素，如果覆盖数等于矩阵维度则完成
	 *
	 * @param assignment 分配结果
	 * @param distMatrix 代价矩阵
	 * @param starMatrix 星标矩阵
	 * @param newStarMatrix 新星标矩阵
	 * @param primeMatrix 撇标矩阵
	 * @param coveredColumns 列覆盖标记数组
	 * @param coveredRows 行覆盖标记数组
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 * @param minDim 矩阵最小维度
	 *
	 * 查找未覆盖的零元素：
	 * - 如果不存在，说明找到了最优解
	 * - 如果存在，对其进行撇标标记，继续步骤4
	 */
	void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

	/**
	 * @brief 步骤4：构建交替路径，更新星标和撇标
	 *
	 * @param assignment 分配结果
	 * @param distMatrix 代价矩阵
	 * @param starMatrix 星标矩阵
	 * @param newStarMatrix 新星标矩阵
	 * @param primeMatrix 撇标矩阵
	 * @param coveredColumns 列覆盖标记数组
	 * @param coveredRows 行覆盖标记数组
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 * @param minDim 矩阵最小维度
	 * @param row 当前行索引
	 * @param col 当前列索引
	 *
	 * 从一个未覆盖的撇标零开始，构建交替路径（星标-撇标-星标-...）
	 * 沿路径交换星标和撇标，增加星标数量
	 */
	void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);

	/**
	 * @brief 步骤5：修改代价矩阵，创建新的零元素
	 *
	 * @param assignment 分配结果
	 * @param distMatrix 代价矩阵
	 * @param starMatrix 星标矩阵
	 * @param newStarMatrix 新星标矩阵
	 * @param primeMatrix 撇标矩阵
	 * @param coveredColumns 列覆盖标记数组
	 * @param coveredRows 行覆盖标记数组
	 * @param nOfRows 矩阵行数
	 * @param nOfColumns 矩阵列数
	 * @param minDim 矩阵最小维度
	 *
	 * 找到未覆盖元素中的最小值h：
	 * - 未覆盖的元素减去h
	 * - 同时被行和列覆盖的元素加上h
	 * 这样可以在不改变最优解的情况下创建新的零元素
	 */
	void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
};

#endif