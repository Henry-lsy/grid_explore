#include "GridPartitioningAndTSP.h"
#include <iostream>

int main() {
    int length = 100;
    int width = 100;
    int rows = 10;
    int cols = 10;
    int numPartitions = 5;

    // 创建网格
    auto grid = createGrid(length, width, rows, cols);

    // 选择n个不同的起点
    std::vector<Point> startPoints = { {0, 0}, {90, 0}, {0, 90}, {90, 90}, {45, 45} };

    // 将网格随机分成n份，每份包含一个起点
    auto partitions = randomPartitionGrid(grid, startPoints);

    // 计算每个分区的最优TSP路径
    for (int i = 0; i < partitions.size(); ++i) {
        auto bestPath = solveTSP(partitions[i]);
        std::cout << "Best path for partition " << i + 1 << ":\n";
        for (const auto& point : bestPath) {
            std::cout << "(" << point.x << ", " << point.y << ") -> ";
        }
        std::cout << "End\n\n";
    }

    return 0;
}
