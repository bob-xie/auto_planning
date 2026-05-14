#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include "planning/DWAPlanner.h"
#include "common/VehicleState.h"
#include "common/Path.h"

int main(int argc, char** argv) {
    std::cout << "=== 测试 DWA 算法 ===" << std::endl;
    
    AD::Planning::DWAPlanner planner;
    AD::Planning::DWAConfig config;
    config.max_speed = 10.0;
    config.min_speed = 0.0;
    config.max_steer_rate = 0.5;
    planner.init(config);
    
    AD::VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    start.vx = 5.0;
    start.wz = 0.0;
    
    AD::Path reference_path;
    for (double i = 0; i <= 50; i += 2) {
        reference_path.addPoint(i, i * 0.5, 0.0, 0.0, 5.0);
    }
    
    std::vector<AD::Planning::ObstacleInfo> obstacles;
    AD::Planning::ObstacleInfo obs;
    obs.x = 20.0;
    obs.y = 10.0;
    obs.radius = 1.0;
    obs.velocity = 0.0;
    obstacles.push_back(obs);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    AD::Path path = planner.plan(start, reference_path, obstacles);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    std::cout << "算法名称: DWA" << std::endl;
    std::cout << "路径长度: " << path.points.size() << " 个点" << std::endl;
    std::cout << "规划时间: " << duration << " ms" << std::endl;
    std::cout << "目标速度: " << planner.getTargetSpeed() << " m/s" << std::endl;
    
    std::cout << "DWA 测试完成" << std::endl;
    return 0;
}