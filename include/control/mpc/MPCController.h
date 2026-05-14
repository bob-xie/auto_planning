#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "../ControllerBase.h"
#include <Eigen/Dense>
#include <vector>

namespace AD {
namespace Control {

struct MPCControllerParams {
    int prediction_horizon = 10;
    int control_horizon = 5;
    
    double dt = 0.1;
    
    double wheel_base = 2.8;
    double front_wheel_base = 1.4;
    double rear_wheel_base = 1.4;
    
    double max_steer = 0.5236;
    double min_steer = -0.5236;
    double max_steer_rate = 0.3;
    double min_steer_rate = -0.3;
    
    double max_acceleration = 2.0;
    double min_acceleration = -3.0;
    
    double max_velocity = 30.0;
    double min_velocity = 0.0;
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
    
    double Q_cte = 10.0;
    double Q_heading = 1.0;
    double Q_velocity = 1.0;
    
    double R_steer = 1.0;
    double R_accel = 1.0;
};

struct ReferencePoint {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    double curvature = 0.0;
    double s = 0.0;
};

class MPCController : public ControllerBase {
public:
    MPCController() = default;
    
    bool init() override;
    
    double compute(const Path& path, const VehicleState& current_state) override;
    
    std::string getName() const override { return "MPC"; }

private:
    MPCControllerParams params_;
    
    double last_steer_ = 0.0;
    double last_accel_ = 0.0;
    
    std::vector<ReferencePoint> reference_trajectory_;
    
    bool updateReferenceTrajectory(const Path& path, const VehicleState& current_state);
    
    Eigen::VectorXd predictState(const Eigen::VectorXd& state, double steer, double accel);
    
    bool solveOptimization(const Eigen::VectorXd& initial_state,
                           std::vector<double>& controls);
    
    double calculateCrossTrackError(const std::vector<ReferencePoint>& trajectory, 
                                    const Eigen::VectorXd& state, int idx);
    
    double calculateHeadingError(const std::vector<ReferencePoint>& trajectory,
                                 const Eigen::VectorXd& state, int idx);
};

}
}

#endif