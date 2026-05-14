#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include "common/Path.h"
#include "common/VehicleState.h"
#include <vector>
#include <Eigen/Dense>

namespace AD {
namespace Planning {

struct ObstacleInfo {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    double velocity = 0.0;
};

struct DWAConfig {
    double max_speed = 15.0;
    double min_speed = -5.0;
    double max_accel = 2.0;
    double max_decel = 4.0;
    double max_steer = 0.5236;
    double max_steer_rate = 0.3;
    double dt = 0.1;
    double predict_time = 3.0;
    double goal_weight = 1.0;
    double obstacle_weight = 10.0;
    double speed_weight = 0.1;
    double wheelbase = 2.8;
};

struct DWATrajectory {
    std::vector<Eigen::Vector3d> points;
    double cost = 0.0;
    double final_speed = 0.0;
};

class DWAPlanner {
public:
    DWAPlanner();
    
    void init(const DWAConfig& config);
    
    void setConfig(const DWAConfig& config);
    
    Path plan(const VehicleState& current_state, 
              const Path& reference_path,
              const std::vector<ObstacleInfo>& obstacles);
    
    double getTargetSpeed() const;
    
private:
    DWAConfig config_;
    double target_speed_ = 0.0;
    
    DWATrajectory generateTrajectory(double v, double w, const VehicleState& state);
    double calculateCost(const DWATrajectory& traj, const Path& path, 
                         const std::vector<ObstacleInfo>& obstacles);
    double distanceToObstacle(const Eigen::Vector3d& point, 
                               const std::vector<ObstacleInfo>& obstacles);
    double distanceToPath(const Eigen::Vector3d& point, const Path& path);
    DWATrajectory dwa(const VehicleState& state, const Path& path, 
                      const std::vector<ObstacleInfo>& obstacles);
};

}
}

#endif