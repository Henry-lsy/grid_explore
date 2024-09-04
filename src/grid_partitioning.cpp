#include "grid_explorer/grid_partitioning.h"
#include <iostream>

double GridPartitioning::distance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void GridPartitioning::createGrid(int length, int width, int rows, int cols) {
    grid_.resize(rows, std::vector<Point>(cols));
    int cellWidth = length / cols;
    int cellHeight = width / rows;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            grid_[i][j] = { j * cellWidth, i * cellHeight };
        }
    }
}

void GridPartitioning::setStartPositions(const std::vector<Point>& start_points)
{
    start_points_ = start_points;
}

std::vector<std::vector<Point>> GridPartitioning::getGridSequences()
{
    auto partitions = randomPartitionGrid(grid_, start_points_);
    std::vector<std::vector<Point>> grid_paths;
    // grid_paths.resize(partitions.size());
    // 计算每个分区的最优TSP路径
    for (int i = 0; i < partitions.size(); ++i) {
        grid_paths.push_back(solveTSP(partitions[i]));
    }
    return grid_paths;
}

std::vector<std::vector<Point>> GridPartitioning::randomPartitionGrid(const std::vector<std::vector<Point>>& grid, const std::vector<Point>& startPoints) {
    int rows = grid.size();
    int cols = grid[0].size();
    int n = startPoints.size();

    std::vector<Point> allPoints;
    for (const auto& row : grid) {
        for (const auto& point : row) {
            allPoints.push_back(point);
        }
    }

    std::shuffle(allPoints.begin(), allPoints.end(), std::mt19937{ std::random_device{}() });

    std::vector<std::vector<Point>> partitions(n);

    for (int i = 0; i < n; ++i) {
        partitions[i].push_back(startPoints[i]);
    }

    int currentPartition = 0;
    for (const auto& point : allPoints) {
        if (std::find(startPoints.begin(), startPoints.end(), point) == startPoints.end()) {
            partitions[currentPartition].push_back(point);
            currentPartition = (currentPartition + 1) % n;
        }
    }

    return partitions;
}

// 动态规划解决TSP问题
std::vector<Point> GridPartitioning::solveTSP(const std::vector<Point>& points) {
    int n = points.size();
    if (n == 0) return {};

    std::vector<std::vector<double>> dist(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            dist[i][j] = distance(points[i], points[j]);
        }
    }

    std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, std::numeric_limits<double>::max()));
    std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));

    dp[1][0] = 0;

    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int u = 0; u < n; ++u) {
            if (mask & (1 << u)) {
                for (int v = 0; v < n; ++v) {
                    if (!(mask & (1 << v))) {
                        double newDist = dp[mask][u] + dist[u][v];
                        if (newDist < dp[mask | (1 << v)][v]) {
                            dp[mask | (1 << v)][v] = newDist;
                            parent[mask | (1 << v)][v] = u;
                        }
                    }
                }
            }
        }
    }

    double bestDist = std::numeric_limits<double>::max();
    int last = -1;
    for (int u = 1; u < n; ++u) {
        double newDist = dp[(1 << n) - 1][u] + dist[u][0];
        if (newDist < bestDist) {
            bestDist = newDist;
            last = u;
        }
    }

    std::vector<Point> path;
    int mask = (1 << n) - 1;
    while (last != -1) {
        path.push_back(points[last]);
        int prev = parent[mask][last];
        mask ^= (1 << last);
        last = prev;
    }
    std::reverse(path.begin(), path.end());
    path.push_back(points[0]);  // Return to the starting point

    return path;
}