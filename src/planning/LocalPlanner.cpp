#include "planning/LocalPlanner.h"
#include <cmath>

namespace AD {
namespace Planning {

bool LocalPlanner::init() {
    target_speed_ = 10.0;
    obstacle_ahead_ = false;
    planner_type_ = LocalPlannerType::DWA;
    
    DWAConfig dwa_config;
    dwa_planner_.init(dwa_config);
    
    LatticeConfig lattice_config;
    lattice_planner_.init(lattice_config);
    
    return true;
}

void LocalPlanner::setPlannerType(LocalPlannerType type) {
    planner_type_ = type;
}

LocalPlannerType LocalPlanner::getPlannerType() const {
    return planner_type_;
}

std::string LocalPlanner::getCurrentPlannerName() const {
    switch (planner_type_) {
        case LocalPlannerType::DWA:
            return "DWA";
        case LocalPlannerType::LATTICE:
            return "Lattice";
        default:
            return "Unknown";
    }
}

std::vector<ObstacleInfo> LocalPlanner::detectObstacles(const Perception::LiDARData& lidar_data) {
    std::vector<ObstacleInfo> obstacles;
    
    if (!lidar_data.cloud) {
        return obstacles;
    }
    
    for (size_t i = 0; i < lidar_data.cloud->size(); i += 100) {
        const auto& point = lidar_data.cloud->points[i];
        
        if (point.x > 0 && point.x < 50 && std::abs(point.y) < 10) {
            ObstacleInfo obs;
            obs.x = point.x;
            obs.y = point.y;
            obs.radius = 1.0;
            obs.velocity = 0.0;
            obstacles.push_back(obs);
        }
    }
    
    return obstacles;
}

std::vector<LatticeObstacle> LocalPlanner::convertToLatticeObstacles(const std::vector<ObstacleInfo>& obstacles) {
    std::vector<LatticeObstacle> lattice_obstacles;
    for (const auto& obs : obstacles) {
        LatticeObstacle lo;
        lo.x = obs.x;
        lo.y = obs.y;
        lo.radius = obs.radius;
        lo.velocity = obs.velocity;
        lattice_obstacles.push_back(lo);
    }
    return lattice_obstacles;
}

Path LocalPlanner::replan(const Path& original_path, 
                          const std::vector<ObstacleInfo>& obstacles) {
    if (obstacles.empty()) {
        target_speed_ = 15.0;
        obstacle_ahead_ = false;
        return original_path;
    }
    
    obstacle_ahead_ = true;
    
    VehicleState current_state;
    current_state.x = 0.0;
    current_state.y = 0.0;
    current_state.yaw = 0.0;
    current_state.vx = target_speed_;
    
    Path new_path;
    
    switch (planner_type_) {
        case LocalPlannerType::DWA:
            new_path = dwa_planner_.plan(current_state, original_path, obstacles);
            target_speed_ = dwa_planner_.getTargetSpeed();
            break;
        case LocalPlannerType::LATTICE: {
            auto lattice_obstacles = convertToLatticeObstacles(obstacles);
            new_path = lattice_planner_.plan(current_state, original_path, lattice_obstacles);
            if (!new_path.points.empty()) {
                target_speed_ = new_path.points.back().velocity;
            }
            break;
        }
        default:
            new_path = dwa_planner_.plan(current_state, original_path, obstacles);
            target_speed_ = dwa_planner_.getTargetSpeed();
            break;
    }
    
    if (new_path.points.empty()) {
        return original_path;
    }
    
    return new_path;
}

double LocalPlanner::getTargetSpeed() const {
    return target_speed_;
}

}
}