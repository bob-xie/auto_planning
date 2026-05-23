#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include "common/Path.h"
#include "common/VehicleState.h"
#include <vector>
#include <Eigen/Dense>

namespace AD {
namespace Planning {

struct LatticeObstacle {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    double velocity = 0.0;
};

struct LatticeConfig {
    double lane_width = 3.5;
    int num_lanes = 5;
    double prediction_time = 5.0;
    double dt = 0.1;
    double max_speed = 20.0;
    double min_speed = 0.0;
    double max_accel = 2.0;
    double max_jerk = 1.0;
    double wheelbase = 2.8;
    
    double path_weight = 1.0;
    double speed_weight = 0.5;
    double jerk_weight = 2.0;
    double obstacle_weight = 10.0;
};

struct LatticeTrajectory {
    std::vector<Eigen::VectorXd> states;
    std::vector<double> controls;
    double cost = 0.0;
    double final_speed = 0.0;
    int target_lane = 0;
};

class LatticePlanner {
public:
    LatticePlanner();
    
    void init(const LatticeConfig& config);
    
    Path plan(const VehicleState& current_state, 
              const Path& reference_path,
              const std::vector<LatticeObstacle>& obstacles);
    
    void setConfig(const LatticeConfig& config);
    
private:
    LatticeConfig config_;
    
    std::vector<LatticeTrajectory> generateTrajectories(const VehicleState& state, 
                                                        const Path& reference_path);
    
    LatticeTrajectory generateLaneChangeTrajectory(const VehicleState& state, 
                                                    int target_lane,
                                                    const Path& reference_path);
    
    double calculateCost(const LatticeTrajectory& traj, 
                        const Path& reference_path,
                        const std::vector<LatticeObstacle>& obstacles);
    
    double distanceToReference(const Eigen::VectorXd& state, const Path& reference_path);
    
    bool checkCollision(const LatticeTrajectory& traj, 
                        const std::vector<LatticeObstacle>& obstacles);
    
    Eigen::VectorXd simulateVehicle(const Eigen::VectorXd& state, 
                                    double steering, double acceleration, double dt);
};

}
}

#endif