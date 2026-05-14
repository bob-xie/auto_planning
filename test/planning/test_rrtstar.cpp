#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include "planning/RRTPlanner.h"
#include "common/VehicleState.h"
#include "common/Path.h"

int main(int argc, char** argv) {
    std::cout << "=== 测试 RRT* 算法 ===" << std::endl;
    
    AD::Planning::RRTStarPlanner planner;
    planner.init(1.0, 0.5);
    
    AD::VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    
    AD::VehicleState goal;
    goal.x = 50.0;
    goal.y = 50.0;
    goal.yaw = 0.0;
    
    std::vector<AD::Planning::Obstacle> obstacles;
    obstacles.emplace_back(20.0, 20.0, 2.0);
    obstacles.emplace_back(40.0, 30.0, 3.0);
    
    planner.setObstacles(obstacles);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    AD::Path path = planner.plan(start, goal);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    std::cout << "算法名称: " << planner.getName() << std::endl;
    std::cout << "路径长度: " << path.points.size() << " 个点" << std::endl;
    std::cout << "规划时间: " << duration << " ms" << std::endl;
    
    if (!path.points.empty()) {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.points.size(); i++) {
            double dx = path.points[i].x - path.points[i-1].x;
            double dy = path.points[i].y - path.points[i-1].y;
            total_distance += std::sqrt(dx*dx + dy*dy);
        }
        std::cout << "路径总距离: " << total_distance << " m" << std::endl;
    }
    
    std::cout << "RRT* 测试完成" << std::endl;
    return 0;
}