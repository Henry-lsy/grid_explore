#ifndef GRID_PARTITIONING_AND_TSP_H
#define GRID_PARTITIONING_AND_TSP_H

#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <limits>

// 代表一个点
struct Point {
    int x;
    int y;

    // 重载 == 操作符，用于比较两个 Point 对象
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// 计算两个点之间的欧几里得距离
double distance(const Point& p1, const Point& p2);

// 将区域划分为网格
std::vector<std::vector<Point>> createGrid(int length, int width, int rows, int cols);

// 将网格随机分成n份，每份包含一个起点
std::vector<std::vector<Point>> randomPartitionGrid(const std::vector<std::vector<Point>>& grid, const std::vector<Point>& startPoints);

// 解决TSP问题，返回最佳路径
std::vector<Point> solveTSP(const std::vector<Point>& points);

#endif // GRID_PARTITIONING_AND_TSP_H
