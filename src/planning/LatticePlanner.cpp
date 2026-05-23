#include "planning/LatticePlanner.h"
#include <cmath>
#include <algorithm>

namespace AD {
namespace Planning {

LatticePlanner::LatticePlanner() {
    LatticeConfig config;
    init(config);
}

void LatticePlanner::init(const LatticeConfig& config) {
    config_ = config;
}

void LatticePlanner::setConfig(const LatticeConfig& config) {
    config_ = config;
}

Eigen::VectorXd LatticePlanner::simulateVehicle(const Eigen::VectorXd& state, 
                                                double steering, double acceleration, double dt) {
    Eigen::VectorXd new_state(6);
    double x = state(0);
    double y = state(1);
    double yaw = state(2);
    double v = state(3);
    double a = state(4);
    double wz = state(5);
    
    double new_v = v + a * dt;
    new_v = std::max(config_.min_speed, std::min(config_.max_speed, new_v));
    
    double new_x = x + v * cos(yaw) * dt;
    double new_y = y + v * sin(yaw) * dt;
    double new_yaw = yaw + wz * dt;
    double new_a = acceleration;
    double new_wz = (v / config_.wheelbase) * tan(steering);
    
    new_state << new_x, new_y, new_yaw, new_v, new_a, new_wz;
    return new_state;
}

double LatticePlanner::distanceToReference(const Eigen::VectorXd& state, const Path& reference_path) {
    if (reference_path.points.empty()) return 0.0;
    
    double min_dist = 1e9;
    for (const auto& pt : reference_path.points) {
        double dx = state(0) - pt.x;
        double dy = state(1) - pt.y;
        double dist = sqrt(dx * dx + dy * dy);
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

bool LatticePlanner::checkCollision(const LatticeTrajectory& traj, 
                                    const std::vector<LatticeObstacle>& obstacles) {
    for (const auto& state : traj.states) {
        double x = state(0);
        double y = state(1);
        
        for (const auto& obs : obstacles) {
            double dx = x - obs.x;
            double dy = y - obs.y;
            double dist = sqrt(dx * dx + dy * dy);
            
            if (dist < obs.radius + 1.0) {
                return true;
            }
        }
    }
    return false;
}

LatticeTrajectory LatticePlanner::generateLaneChangeTrajectory(const VehicleState& state, 
                                                               int target_lane,
                                                               const Path& reference_path) {
    LatticeTrajectory traj;
    
    Eigen::VectorXd current_state(6);
    current_state << state.x, state.y, state.yaw, state.vx, state.ax, state.wz;
    
    traj.states.push_back(current_state);
    
    double current_v = state.vx;
    double current_a = 0.0;
    
    double lane_offset = (target_lane - 2) * config_.lane_width;
    double target_y = state.y + lane_offset;
    
    for (double t = 0; t < config_.prediction_time; t += config_.dt) {
        double y_error = target_y - current_state(1);
        double steering = 0.1 * y_error;
        steering = std::max(-config_.wheelbase, std::min(config_.wheelbase, steering));
        
        double v_error = config_.max_speed - current_v;
        current_a = 0.5 * v_error;
        current_a = std::max(-config_.max_accel, std::min(config_.max_accel, current_a));
        
        current_state = simulateVehicle(current_state, steering, current_a, config_.dt);
        traj.states.push_back(current_state);
        traj.controls.push_back(steering);
        
        current_v = current_state(3);
    }
    
    traj.final_speed = current_v;
    traj.target_lane = target_lane;
    
    return traj;
}

std::vector<LatticeTrajectory> LatticePlanner::generateTrajectories(const VehicleState& state, 
                                                                    const Path& reference_path) {
    std::vector<LatticeTrajectory> trajectories;
    
    for (int lane = 0; lane < config_.num_lanes; ++lane) {
        LatticeTrajectory traj = generateLaneChangeTrajectory(state, lane, reference_path);
        trajectories.push_back(traj);
    }
    
    return trajectories;
}

double LatticePlanner::calculateCost(const LatticeTrajectory& traj, 
                                     const Path& reference_path,
                                     const std::vector<LatticeObstacle>& obstacles) {
    double cost = 0.0;
    
    if (checkCollision(traj, obstacles)) {
        return 1e9;
    }
    
    for (const auto& state : traj.states) {
        cost += config_.path_weight * distanceToReference(state, reference_path);
    }
    
    cost += config_.speed_weight * (config_.max_speed - traj.final_speed);
    
    for (size_t i = 1; i < traj.controls.size(); ++i) {
        double jerk = (traj.controls[i] - traj.controls[i-1]) / config_.dt;
        cost += config_.jerk_weight * jerk * jerk;
    }
    
    return cost;
}

Path LatticePlanner::plan(const VehicleState& current_state, 
                          const Path& reference_path,
                          const std::vector<LatticeObstacle>& obstacles) {
    auto trajectories = generateTrajectories(current_state, reference_path);
    
    LatticeTrajectory best_traj;
    best_traj.cost = 1e9;
    
    for (auto& traj : trajectories) {
        traj.cost = calculateCost(traj, reference_path, obstacles);
        if (traj.cost < best_traj.cost) {
            best_traj = traj;
        }
    }
    
    Path new_path;
    for (const auto& state : best_traj.states) {
        new_path.addPoint(state(0), state(1), 0.0, state(2), best_traj.final_speed);
    }
    
    return new_path;
}

}
}