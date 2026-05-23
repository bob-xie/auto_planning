#include "planning/DWAPlanner.h"
#include <cmath>
#include <algorithm>

namespace AD {
namespace Planning {

DWAPlanner::DWAPlanner() {
    DWAConfig config;
    init(config);
}

void DWAPlanner::init(const DWAConfig& config) {
    config_ = config;
    target_speed_ = 10.0;
}

void DWAPlanner::setConfig(const DWAConfig& config) {
    config_ = config;
}

DWATrajectory DWAPlanner::generateTrajectory(double v, double w, const VehicleState& state) {
    DWATrajectory traj;
    double x = state.x;
    double y = state.y;
    double yaw = state.yaw;
    double current_v = state.vx;
    
    for (double t = 0; t < config_.predict_time; t += config_.dt) {
        x += current_v * cos(yaw) * config_.dt;
        y += current_v * sin(yaw) * config_.dt;
        yaw += (current_v / config_.wheelbase) * tan(w) * config_.dt;
        current_v += v * config_.dt;
        
        current_v = std::max(config_.min_speed, std::min(config_.max_speed, current_v));
        
        traj.points.push_back(Eigen::Vector3d(x, y, yaw));
    }
    
    traj.final_speed = current_v;
    return traj;
}

double DWAPlanner::distanceToObstacle(const Eigen::Vector3d& point, 
                                       const std::vector<ObstacleInfo>& obstacles) {
    double min_dist = 1e9;
    for (const auto& obs : obstacles) {
        double dx = point(0) - obs.x;
        double dy = point(1) - obs.y;
        double dist = sqrt(dx * dx + dy * dy) - obs.radius;
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

double DWAPlanner::distanceToPath(const Eigen::Vector3d& point, const Path& path) {
    if (path.points.empty()) return 0.0;
    
    double min_dist = 1e9;
    for (const auto& pt : path.points) {
        double dx = point(0) - pt.x;
        double dy = point(1) - pt.y;
        double dist = sqrt(dx * dx + dy * dy);
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

double DWAPlanner::calculateCost(const DWATrajectory& traj, const Path& path, 
                                 const std::vector<ObstacleInfo>& obstacles) {
    double cost = 0.0;
    
    if (!traj.points.empty()) {
        const auto& final_point = traj.points.back();
        cost += config_.goal_weight * distanceToPath(final_point, path);
    }
    
    double min_dist_to_obs = 1e9;
    for (const auto& point : traj.points) {
        min_dist_to_obs = std::min(min_dist_to_obs, distanceToObstacle(point, obstacles));
    }
    if (min_dist_to_obs < 0.5) {
        cost += config_.obstacle_weight / (min_dist_to_obs + 0.01);
    }
    
    cost += config_.speed_weight * (config_.max_speed - traj.final_speed);
    
    return cost;
}

DWATrajectory DWAPlanner::dwa(const VehicleState& state, const Path& path, 
                              const std::vector<ObstacleInfo>& obstacles) {
    DWATrajectory best_traj;
    best_traj.cost = 1e9;
    
    int num_samples = 20;
    
    for (int i = 0; i < num_samples; ++i) {
        double v = config_.min_speed + (config_.max_speed - config_.min_speed) * (i / (double)(num_samples - 1));
        
        for (int j = 0; j < num_samples; ++j) {
            double w = -config_.max_steer + 2 * config_.max_steer * (j / (double)(num_samples - 1));
            
            DWATrajectory traj = generateTrajectory(v, w, state);
            traj.cost = calculateCost(traj, path, obstacles);
            
            if (traj.cost < best_traj.cost) {
                best_traj = traj;
            }
        }
    }
    
    return best_traj;
}

Path DWAPlanner::plan(const VehicleState& current_state, 
                      const Path& reference_path,
                      const std::vector<ObstacleInfo>& obstacles) {
    if (obstacles.empty()) {
        target_speed_ = config_.max_speed;
        return reference_path;
    }
    
    DWATrajectory best_traj = dwa(current_state, reference_path, obstacles);
    
    Path new_path;
    for (const auto& point : best_traj.points) {
        new_path.addPoint(point(0), point(1), 0.0, point(2), best_traj.final_speed);
    }
    
    target_speed_ = std::max(5.0, best_traj.final_speed);
    
    if (new_path.points.empty()) {
        return reference_path;
    }
    
    return new_path;
}

double DWAPlanner::getTargetSpeed() const {
    return target_speed_;
}

}
}