#include "control/lateral/LateralController.h"
#include <cmath>

namespace AD {
namespace Control {

bool LateralController::init() {
    params_.k = 0.5;
    params_.k_soft = 1.0;
    params_.max_steer = 0.5236;
    params_.min_steer = -0.5236;
    return true;
}

double LateralController::compute(const Path& path, const VehicleState& current_state) {
    if (path.points.empty()) return 0.0;
    
    double cte = calculateCrossTrackError(path, current_state);
    double target_yaw = calculateTargetHeading(path, current_state);
    
    double heading_error = target_yaw - current_state.yaw;
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;
    
    double steer = heading_error + 
                   atan2(params_.k * cte, params_.k_soft + current_state.vx);
    
    steer = std::max(params_.min_steer, std::min(params_.max_steer, steer));
    
    return steer;
}

double LateralController::calculateCrossTrackError(const Path& path, 
                                                    const VehicleState& state) {
    double min_dist = 1e9;
    for (const auto& point : path.points) {
        double dx = point.x - state.x;
        double dy = point.y - state.y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

double LateralController::calculateTargetHeading(const Path& path, 
                                                  const VehicleState& state) {
    if (path.points.size() < 2) return 0.0;
    
    for (size_t i = 0; i < path.points.size() - 1; ++i) {
        const auto& p1 = path.points[i];
        const auto& p2 = path.points[i + 1];
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        
        double dist_to_p1 = sqrt((state.x - p1.x) * (state.x - p1.x) + 
                                 (state.y - p1.y) * (state.y - p1.y));
        double dist_to_p2 = sqrt((state.x - p2.x) * (state.x - p2.x) + 
                                 (state.y - p2.y) * (state.y - p2.y));
        
        if (dist_to_p1 < dist_to_p2) {
            return p1.yaw;
        }
    }
    
    return path.points[path.points.size() - 1].yaw;
}

}
}