#ifndef GRID_PARTITIONING_ROS_H
#define GRID_PARTITIONING_ROS_H

#include <vector>
#include <memory>

#include <ros/ros.h>

#include "grid_explorer/grid_partitioning.h"


class GridPartitioningROS
{
    public:
    GridPartitioningROS(ros::NodeHandle& nh): nh_(nh),grid_partitioning_ptr_(std::make_shared<GridPartitioning>())
    {
        nh_.param("length", length_, 100);
        nh_.param("width", width_, 100);
        nh_.param("rows", rows_, 10);
        nh_.param("cols", cols_, 10);
        nh_.param("num_partitions", numPartitions_, 5);

        grid_partitioning_ptr_ -> createGrid(length_, width_, rows_, cols_);
    }

    void setStartPositions(const std::vector<Point> & start_positions)
    {
        grid_partitioning_ptr_->setStartPositions(start_positions);
    }

    std::vector<std::vector<Point>> generateGridSequences()
    {
        return grid_partitioning_ptr_->getGridSequences();
    }

    ~GridPartitioningROS() = default;

    private:
        ros::NodeHandle nh_;

        std::shared_ptr<GridPartitioning> grid_partitioning_ptr_;
        int length_{}, width_{}, rows_{}, cols_{}, numPartitions_{};

};
#endif // GRID_PARTITIONING_ROS_H
