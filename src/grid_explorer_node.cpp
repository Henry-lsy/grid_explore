#include <iostream>
#include <ros/ros.h>

#include "grid_explorer/grid_partitioning_ros.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "grid_explorer_node");
    ros::NodeHandle nh("~");
    GridPartitioningROS grid_partitioning_ros(nh);

    std::vector<Point> startPoints = { {0, 0}, {90, 0}, {0, 90}, {90, 90}, {45, 45} };
   
    // 创建网格
    grid_partitioning_ros.setStartPositions(startPoints);

    std::vector<std::vector<Point>> explorer_paths;
    // 选择n个不同的起点
    explorer_paths = grid_partitioning_ros.generateGridSequences();

    // 输出每个分区的最优TSP路径
    for (int i = 0; i < explorer_paths.size(); ++i) {
        std::cout << "Best path for partition " << i + 1 << ":\n";
        for (const auto& point : explorer_paths[i]) {
            std::cout << "(" << point.x << ", " << point.y << ") -> ";
        }
        std::cout << "End\n\n";
    }

    return 0;
}
