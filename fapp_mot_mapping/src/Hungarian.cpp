/**
 * @file Hungarian.cpp
 * @brief 匈牙利算法实现 - 用于解决分配问题（Assignment Problem）
 * @details 实现了Munkres算法（也称为匈牙利算法），用于在O(n^3)时间内求解
 *          最优分配问题。该算法常用于目标跟踪中的数据关联问题。
 */

#include "Hungarian.h"
#include <cfloat>

// 构造函数
HungarianAlgorithm::HungarianAlgorithm(){}

// 析构函数
HungarianAlgorithm::~HungarianAlgorithm(){}


//********************************************************//
// 求解分配问题的单一函数封装
//********************************************************//
/**
 * @brief 求解分配问题的主函数接口
 * @param DistMatrix 输入的代价矩阵（MxN维），DistMatrix[i][j]表示将第i个任务分配给第j个资源的代价
 * @param Assignment 输出的分配结果向量，Assignment[i]表示第i个任务被分配给哪个资源
 * @return 返回最优分配的总代价
 * @details 该函数将二维代价矩阵转换为一维数组，调用核心算法求解，然后将结果转换回向量形式
 */
double HungarianAlgorithm::Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment)
{
	unsigned int nRows = DistMatrix.size();      // 行数（任务数）
	unsigned int nCols = DistMatrix[0].size();   // 列数（资源数）

	double *distMatrixIn = new double[nRows * nCols];  // 一维数组形式的代价矩阵
	int *assignment = new int[nRows];                   // 分配结果数组
	double cost = 0.0;                                  // 总代价

	// 填充代价矩阵。注意索引是"i + nRows * j"
	// 这里大小为MxN的代价矩阵被定义为包含N*M个元素的双精度数组
	// 在求解函数中，矩阵以MATLAB内部的列优先顺序存储
	// (例如矩阵[1 2; 3 4]将被存储为向量[1 3 2 4]，而不是[1 2 3 4])
	for (unsigned int i = 0; i < nRows; i++)
		for (unsigned int j = 0; j < nCols; j++)
			distMatrixIn[i + nRows * j] = DistMatrix[i][j];

	// 调用核心求解函数
	assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

	// 将结果从数组转换为向量
	Assignment.clear();
	for (unsigned int r = 0; r < nRows; r++)
		Assignment.push_back(assignment[r]);

	// 释放动态分配的内存
	delete[] distMatrixIn;
	delete[] assignment;
	return cost;
}


//********************************************************//
// 使用Munkres算法（也称为匈牙利算法）求解分配问题的最优解
//********************************************************//
/**
 * @brief 匈牙利算法的核心求解函数
 * @param assignment 输出参数，存储分配结果
 * @param cost 输出参数，存储最优解的总代价
 * @param distMatrixIn 输入的代价矩阵（一维数组形式）
 * @param nOfRows 矩阵的行数
 * @param nOfColumns 矩阵的列数
 * @details 该函数实现了完整的匈牙利算法，包括：
 *          1. 矩阵预处理（行/列约简）
 *          2. 寻找独立零元素
 *          3. 覆盖零元素
 *          4. 增广路径调整
 *          5. 最小未覆盖元素调整
 */
void HungarianAlgorithm::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
	double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
	bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
	int nOfElements, minDim, row, col;

	/* 初始化 */
	*cost = 0;
	for (row = 0; row<nOfRows; row++)
		assignment[row] = -1;  // -1表示未分配

	/* 生成距离矩阵的工作副本 */
	/* 检查所有矩阵元素是否为非负数 */
	nOfElements = nOfRows * nOfColumns;
	distMatrix = (double *)malloc(nOfElements * sizeof(double));
	distMatrixEnd = distMatrix + nOfElements;

	for (row = 0; row<nOfElements; row++)
	{
		value = distMatrixIn[row];
		if (value < 0)
			cerr << "All matrix elements have to be non-negative." << endl;
		distMatrix[row] = value;
	}


	/* 内存分配 */
	coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));  // 列覆盖标记数组
	coveredRows = (bool *)calloc(nOfRows, sizeof(bool));        // 行覆盖标记数组
	starMatrix = (bool *)calloc(nOfElements, sizeof(bool));     // 星标零元素矩阵（表示当前分配）
	primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));    // 撇标零元素矩阵（表示候选分配）
	newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool));  // 新的星标矩阵（在步骤4中使用）

	/* 预处理步骤 */
	if (nOfRows <= nOfColumns)  // 任务数 <= 资源数的情况
	{
		minDim = nOfRows;

		// 对每一行进行约简：减去行最小值
		for (row = 0; row<nOfRows; row++)
		{
			/* 找到该行中的最小元素 */
			distMatrixTemp = distMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nOfRows;
			while (distMatrixTemp < distMatrixEnd)
			{
				value = *distMatrixTemp;
				if (value < minValue)
					minValue = value;
				distMatrixTemp += nOfRows;
			}

			/* 从该行的每个元素中减去最小值 */
			distMatrixTemp = distMatrix + row;
			while (distMatrixTemp < distMatrixEnd)
			{
				*distMatrixTemp -= minValue;
				distMatrixTemp += nOfRows;
			}
		}

		/* 步骤1和2a：为每一行找到第一个零元素并标记为星标零 */
		for (row = 0; row<nOfRows; row++)
			for (col = 0; col<nOfColumns; col++)
				if (abs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)  // 找到零元素（使用DBL_EPSILON避免浮点数比较问题）
					if (!coveredColumns[col])  // 如果该列还未被覆盖
					{
						starMatrix[row + nOfRows*col] = true;  // 标记为星标零
						coveredColumns[col] = true;             // 标记该列已覆盖
						break;
					}
	}
	else /* if(nOfRows > nOfColumns) */  // 任务数 > 资源数的情况
	{
		minDim = nOfColumns;

		// 对每一列进行约简：减去列最小值
		for (col = 0; col<nOfColumns; col++)
		{
			/* 找到该列中的最小元素 */
			distMatrixTemp = distMatrix + nOfRows*col;
			columnEnd = distMatrixTemp + nOfRows;

			minValue = *distMatrixTemp++;
			while (distMatrixTemp < columnEnd)
			{
				value = *distMatrixTemp++;
				if (value < minValue)
					minValue = value;
			}

			/* 从该列的每个元素中减去最小值 */
			distMatrixTemp = distMatrix + nOfRows*col;
			while (distMatrixTemp < columnEnd)
				*distMatrixTemp++ -= minValue;
		}

		/* 步骤1和2a：为每一列找到第一个零元素并标记为星标零 */
		for (col = 0; col<nOfColumns; col++)
			for (row = 0; row<nOfRows; row++)
				if (abs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)  // 找到零元素
					if (!coveredRows[row])  // 如果该行还未被覆盖
					{
						starMatrix[row + nOfRows*col] = true;  // 标记为星标零
						coveredColumns[col] = true;             // 标记该列已覆盖
						coveredRows[row] = true;                // 标记该行已覆盖
						break;
					}
		// 清除行覆盖标记（列覆盖标记保留）
		for (row = 0; row<nOfRows; row++)
			coveredRows[row] = false;

	}

	/* 移动到步骤2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

	/* 计算总代价并移除无效分配 */
	computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);

	/* 释放分配的内存 */
	free(distMatrix);
	free(coveredColumns);
	free(coveredRows);
	free(starMatrix);
	free(primeMatrix);
	free(newStarMatrix);

	return;
}

/********************************************************/
/**
 * @brief 根据星标矩阵构建分配结果向量
 * @param assignment 输出的分配结果数组
 * @param starMatrix 星标零元素矩阵
 * @param nOfRows 行数
 * @param nOfColumns 列数
 * @details 遍历星标矩阵，将每行的星标零位置记录到分配向量中
 */
void HungarianAlgorithm::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
	int row, col;

	// 对每一行，找到其对应的星标零所在的列
	for (row = 0; row<nOfRows; row++)
		for (col = 0; col<nOfColumns; col++)
			if (starMatrix[row + nOfRows*col])  // 找到星标零
			{
#ifdef ONE_INDEXING
				assignment[row] = col + 1; /* MATLAB索引（从1开始） */
#else
				assignment[row] = col;      // C++索引（从0开始）
#endif
				break;
			}
}

/********************************************************/
/**
 * @brief 计算分配方案的总代价
 * @param assignment 分配结果数组
 * @param cost 输出参数，累加总代价
 * @param distMatrix 原始代价矩阵
 * @param nOfRows 行数
 * @details 根据分配结果，累加每个分配的代价值
 */
void HungarianAlgorithm::computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
{
	int row, col;

	// 累加每个任务的分配代价
	for (row = 0; row<nOfRows; row++)
	{
		col = assignment[row];
		if (col >= 0)  // 如果该任务已被分配
			*cost += distMatrix[row + nOfRows*col];  // 累加代价
	}
}

/********************************************************/
/**
 * @brief 步骤2a：覆盖所有包含星标零的列
 * @details 遍历所有列，如果某列包含星标零，则标记该列为已覆盖
 */
void HungarianAlgorithm::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool *starMatrixTemp, *columnEnd;
	int col;

	/* 覆盖每一个包含星标零的列 */
	for (col = 0; col<nOfColumns; col++)
	{
		starMatrixTemp = starMatrix + nOfRows*col;  // 指向该列的起始位置
		columnEnd = starMatrixTemp + nOfRows;        // 指向该列的结束位置
		while (starMatrixTemp < columnEnd){
			if (*starMatrixTemp++)  // 如果找到星标零
			{
				coveredColumns[col] = true;  // 标记该列为已覆盖
				break;
			}
		}
	}

	/* 移动到步骤2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
/**
 * @brief 步骤2b：检查是否找到最优解
 * @details 统计已覆盖的列数，如果等于矩阵的最小维度，说明找到了最优解；
 *          否则继续执行步骤3
 */
void HungarianAlgorithm::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	int col, nOfCoveredColumns;

	/* 统计已覆盖的列数 */
	nOfCoveredColumns = 0;
	for (col = 0; col<nOfColumns; col++)
		if (coveredColumns[col])
			nOfCoveredColumns++;

	if (nOfCoveredColumns == minDim)
	{
		/* 算法结束：找到最优解 */
		buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
	}
	else
	{
		/* 移动到步骤3：需要继续寻找增广路径 */
		step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}

}

/********************************************************/
/**
 * @brief 步骤3：寻找未覆盖的零元素并进行撇标或调整覆盖
 * @details 遍历所有未被覆盖的零元素：
 *          - 如果该零所在行没有星标零，则找到增广路径，转到步骤4
 *          - 如果该零所在行有星标零，则覆盖该行，取消覆盖星标零所在列
 *          如果没有找到未覆盖的零，转到步骤5
 */
void HungarianAlgorithm::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool zerosFound;
	int row, col, starCol;

	zerosFound = true;
	while (zerosFound)
	{
		zerosFound = false;
		// 遍历所有未覆盖的列
		for (col = 0; col<nOfColumns; col++)
			if (!coveredColumns[col])
				// 遍历所有未覆盖的行
				for (row = 0; row<nOfRows; row++)
					if ((!coveredRows[row]) && (abs(distMatrix[row + nOfRows*col]) < DBL_EPSILON))  // 找到未覆盖的零元素
					{
						/* 标记为撇标零 */
						primeMatrix[row + nOfRows*col] = true;

						/* 在当前行中寻找星标零 */
						for (starCol = 0; starCol<nOfColumns; starCol++)
							if (starMatrix[row + nOfRows*starCol])
								break;

						if (starCol == nOfColumns) /* 当前行没有星标零 */
						{
							/* 找到增广路径，移动到步骤4 */
							step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
							return;
						}
						else  // 当前行有星标零
						{
							coveredRows[row] = true;          // 覆盖当前行
							coveredColumns[starCol] = false;  // 取消覆盖星标零所在的列
							zerosFound = true;
							break;
						}
					}
	}

	/* 没有找到未覆盖的零，移动到步骤5 */
	step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
/**
 * @brief 步骤4：构造增广路径并更新星标零
 * @param row 起始撇标零的行索引
 * @param col 起始撇标零的列索引
 * @details 从给定的撇标零开始，构造一条交替路径（撇标零-星标零-撇标零...）
 *          然后沿着这条路径翻转星标零和撇标零，增加星标零的数量
 *          增广路径的构造：
 *          1. 当前撇标零 -> 同列的星标零 -> 同行的撇标零 -> ...
 *          2. 将路径上的撇标零改为星标零，星标零取消星标
 */
void HungarianAlgorithm::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
	int n, starRow, starCol, primeRow, primeCol;
	int nOfElements = nOfRows*nOfColumns;

	/* 生成星标矩阵的临时副本 */
	for (n = 0; n<nOfElements; n++)
		newStarMatrix[n] = starMatrix[n];

	/* 将当前撇标零改为星标零（增广路径的起点） */
	newStarMatrix[row + nOfRows*col] = true;

	/* 在当前列中寻找星标零 */
	starCol = col;
	for (starRow = 0; starRow<nOfRows; starRow++)
		if (starMatrix[starRow + nOfRows*starCol])
			break;

	// 沿着增广路径前进，交替翻转星标零和撇标零
	while (starRow<nOfRows)
	{
		/* 取消该星标零的星标 */
		newStarMatrix[starRow + nOfRows*starCol] = false;

		/* 在当前行中寻找撇标零 */
		primeRow = starRow;
		for (primeCol = 0; primeCol<nOfColumns; primeCol++)
			if (primeMatrix[primeRow + nOfRows*primeCol])
				break;

		/* 将该撇标零改为星标零 */
		newStarMatrix[primeRow + nOfRows*primeCol] = true;

		/* 在当前列中寻找星标零，继续构造增广路径 */
		starCol = primeCol;
		for (starRow = 0; starRow<nOfRows; starRow++)
			if (starMatrix[starRow + nOfRows*starCol])
				break;
	}

	/* 使用临时副本作为新的星标矩阵 */
	/* 删除所有撇标，取消所有行的覆盖 */
	for (n = 0; n<nOfElements; n++)
	{
		primeMatrix[n] = false;           // 清除所有撇标
		starMatrix[n] = newStarMatrix[n]; // 更新星标矩阵
	}
	for (n = 0; n<nOfRows; n++)
		coveredRows[n] = false;  // 取消所有行的覆盖

	/* 移动到步骤2a */
	step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
/**
 * @brief 步骤5：调整代价矩阵以产生新的零元素
 * @details 当没有未覆盖的零元素时，需要修改矩阵来产生新的零：
 *          1. 找到所有未覆盖元素中的最小值h
 *          2. 将h加到所有已覆盖行的元素上
 *          3. 将h从所有未覆盖列的元素中减去
 *          这样可以保证：
 *          - 至少产生一个新的零元素
 *          - 不改变最优解
 *          - 保持所有元素非负
 */
void HungarianAlgorithm::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	double h, value;
	int row, col;

	/* 找到最小的未覆盖元素h */
	h = DBL_MAX;
	for (row = 0; row<nOfRows; row++)
		if (!coveredRows[row])  // 未覆盖的行
			for (col = 0; col<nOfColumns; col++)
				if (!coveredColumns[col])  // 未覆盖的列
				{
					value = distMatrix[row + nOfRows*col];
					if (value < h)
						h = value;  // 更新最小值
				}

	/* 将h加到每个已覆盖行的所有元素上 */
	for (row = 0; row<nOfRows; row++)
		if (coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				distMatrix[row + nOfRows*col] += h;

	/* 从每个未覆盖列的所有元素中减去h */
	for (col = 0; col<nOfColumns; col++)
		if (!coveredColumns[col])
			for (row = 0; row<nOfRows; row++)
				distMatrix[row + nOfRows*col] -= h;

	/* 移动到步骤3：重新寻找未覆盖的零元素 */
	step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}