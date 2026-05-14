#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include "planning/LatticePlanner.h"
#include "common/VehicleState.h"
#include "common/Path.h"

int main(int argc, char** argv) {
    std::cout << "=== 测试 Lattice 算法 ===" << std::endl;
    
    AD::Planning::LatticePlanner planner;
    AD::Planning::LatticeConfig config;
    config.num_lanes = 3;
    config.lane_width = 3.5;
    planner.init(config);
    
    AD::VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    start.vx = 8.0;
    
    AD::Path reference_path;
    for (double i = 0; i <= 100; i += 5) {
        reference_path.addPoint(i, 0.0, 0.0, 0.0, 8.0);
    }
    
    std::vector<AD::Planning::LatticeObstacle> obstacles;
    AD::Planning::LatticeObstacle obs;
    obs.x = 30.0;
    obs.y = 0.0;
    obs.radius = 2.0;
    obs.velocity = 0.0;
    obstacles.push_back(obs);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    AD::Path path = planner.plan(start, reference_path, obstacles);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    std::cout << "算法名称: Lattice" << std::endl;
    std::cout << "路径长度: " << path.points.size() << " 个点" << std::endl;
    std::cout << "规划时间: " << duration << " ms" << std::endl;
    
    if (!path.points.empty()) {
        std::cout << "初始车道位置: y=" << reference_path.points[0].y << std::endl;
        std::cout << "最终车道位置: y=" << path.points.back().y << std::endl;
    }
    
    std::cout << "Lattice 测试完成" << std::endl;
    return 0;
}