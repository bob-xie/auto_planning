#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include "planning/GlobalPlanner.h"
#include "planning/LocalPlanner.h"
#include "planning/AStarPlanner.h"
#include "planning/RRTPlanner.h"
#include "planning/DWAPlanner.h"
#include "planning/LatticePlanner.h"
#include "common/VehicleState.h"
#include "common/Path.h"

namespace AD {
namespace Test {

void testAStarPlanner() {
    std::cout << "\n=== 测试 A* 算法 ===" << std::endl;
    
    Planning::AStarPlanner planner;
    planner.init(0.5, 0.5);
    
    VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    
    VehicleState goal;
    goal.x = 50.0;
    goal.y = 50.0;
    goal.yaw = 0.0;
    
    std::vector<Planning::Obstacle> obstacles;
    obstacles.emplace_back(10.0, 10.0, 2.0);
    obstacles.emplace_back(20.0, 15.0, 1.5);
    obstacles.emplace_back(30.0, 5.0, 2.0);
    
    planner.setObstacles(obstacles);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Path path = planner.plan(start, goal);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
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
        std::cout << "起点: (" << path.points[0].x << ", " << path.points[0].y << ")" << std::endl;
        std::cout << "终点: (" << path.points.back().x << ", " << path.points.back().y << ")" << std::endl;
    }
    
    std::cout << "A* 测试完成" << std::endl;
}

void testRRTPlanner() {
    std::cout << "\n=== 测试 RRT 算法 ===" << std::endl;
    
    Planning::RRTPlanner planner;
    planner.init(1.0, 0.5);
    
    VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    
    VehicleState goal;
    goal.x = 50.0;
    goal.y = 50.0;
    goal.yaw = 0.0;
    
    std::vector<Planning::Obstacle> obstacles;
    obstacles.emplace_back(15.0, 10.0, 3.0);
    obstacles.emplace_back(30.0, 25.0, 2.5);
    
    planner.setObstacles(obstacles);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Path path = planner.plan(start, goal);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
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
    
    std::cout << "RRT 测试完成" << std::endl;
}

void testRRTStarPlanner() {
    std::cout << "\n=== 测试 RRT* 算法 ===" << std::endl;
    
    Planning::RRTStarPlanner planner;
    planner.init(1.0, 0.5);
    
    VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    
    VehicleState goal;
    goal.x = 50.0;
    goal.y = 50.0;
    goal.yaw = 0.0;
    
    std::vector<Planning::Obstacle> obstacles;
    obstacles.emplace_back(20.0, 20.0, 2.0);
    obstacles.emplace_back(40.0, 30.0, 3.0);
    
    planner.setObstacles(obstacles);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Path path = planner.plan(start, goal);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
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
}

void testDWAPlanner() {
    std::cout << "\n=== 测试 DWA 算法 ===" << std::endl;
    
    Planning::DWAPlanner planner;
    Planning::DWAConfig config;
    config.max_speed = 10.0;
    config.min_speed = 0.0;
    config.max_steer_rate = 0.5;
    planner.init(config);
    
    VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    start.vx = 5.0;
    start.wz = 0.0;
    
    Path reference_path;
    for (double i = 0; i <= 50; i += 2) {
        reference_path.addPoint(i, i * 0.5, 0.0, 0.0, 5.0);
    }
    
    std::vector<Planning::ObstacleInfo> obstacles;
    Planning::ObstacleInfo obs;
    obs.x = 20.0;
    obs.y = 10.0;
    obs.radius = 1.0;
    obs.velocity = 0.0;
    obstacles.push_back(obs);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Path path = planner.plan(start, reference_path, obstacles);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    std::cout << "路径长度: " << path.points.size() << " 个点" << std::endl;
    std::cout << "规划时间: " << duration << " ms" << std::endl;
    std::cout << "目标速度: " << planner.getTargetSpeed() << " m/s" << std::endl;
    
    std::cout << "DWA 测试完成" << std::endl;
}

void testLatticePlanner() {
    std::cout << "\n=== 测试 Lattice 算法 ===" << std::endl;
    
    Planning::LatticePlanner planner;
    Planning::LatticeConfig config;
    config.num_lanes = 3;
    config.lane_width = 3.5;
    planner.init(config);
    
    VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    start.vx = 8.0;
    
    Path reference_path;
    for (double i = 0; i <= 100; i += 5) {
        reference_path.addPoint(i, 0.0, 0.0, 0.0, 8.0);
    }
    
    std::vector<Planning::LatticeObstacle> obstacles;
    Planning::LatticeObstacle obs;
    obs.x = 30.0;
    obs.y = 0.0;
    obs.radius = 2.0;
    obs.velocity = 0.0;
    obstacles.push_back(obs);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Path path = planner.plan(start, reference_path, obstacles);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    std::cout << "路径长度: " << path.points.size() << " 个点" << std::endl;
    std::cout << "规划时间: " << duration << " ms" << std::endl;
    
    if (!path.points.empty()) {
        std::cout << "初始车道位置: y=" << reference_path.points[0].y << std::endl;
        std::cout << "最终车道位置: y=" << path.points.back().y << std::endl;
    }
    
    std::cout << "Lattice 测试完成" << std::endl;
}

void testGlobalPlannerSwitching() {
    std::cout << "\n=== 测试全局规划器算法切换 ===" << std::endl;
    
    Planning::GlobalPlanner planner;
    
    VehicleState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    
    planner.setGoal(50.0, 50.0);
    
    planner.setPlannerType(Planning::PlannerType::ASTAR);
    std::cout << "当前算法: " << planner.getCurrentPlannerName() << std::endl;
    Path path1 = planner.plan(state);
    std::cout << "A* 路径点数量: " << path1.points.size() << std::endl;
    
    planner.setPlannerType(Planning::PlannerType::RRT);
    std::cout << "当前算法: " << planner.getCurrentPlannerName() << std::endl;
    Path path2 = planner.plan(state);
    std::cout << "RRT 路径点数量: " << path2.points.size() << std::endl;
    
    planner.setPlannerType(Planning::PlannerType::RRT_STAR);
    std::cout << "当前算法: " << planner.getCurrentPlannerName() << std::endl;
    Path path3 = planner.plan(state);
    std::cout << "RRT* 路径点数量: " << path3.points.size() << std::endl;
    
    std::cout << "全局规划器切换测试完成" << std::endl;
}

void testLocalPlannerSwitching() {
    std::cout << "\n=== 测试局部规划器算法切换 ===" << std::endl;
    
    Planning::LocalPlanner planner;
    
    planner.setPlannerType(Planning::LocalPlannerType::DWA);
    std::cout << "当前局部算法: " << planner.getCurrentPlannerName() << std::endl;
    
    planner.setPlannerType(Planning::LocalPlannerType::LATTICE);
    std::cout << "当前局部算法: " << planner.getCurrentPlannerName() << std::endl;
    
    std::cout << "局部规划器切换测试完成" << std::endl;
}

void runAllTests() {
    std::cout << "=== 路径规划算法测试套件 ===" << std::endl;
    std::cout << "测试时间: " << __DATE__ << " " << __TIME__ << std::endl;
    
    testAStarPlanner();
    testRRTPlanner();
    testRRTStarPlanner();
    testDWAPlanner();
    testLatticePlanner();
    testGlobalPlannerSwitching();
    testLocalPlannerSwitching();
    
    std::cout << "\n=== 所有测试完成 ===" << std::endl;
}

}
}

int main(int argc, char** argv) {
    AD::Test::runAllTests();
    return 0;
}