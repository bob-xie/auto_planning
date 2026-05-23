#include "control/mpc/MPCController.h"
#include <cmath>

namespace AD {
namespace Control {

bool MPCController::init() {
    params_.prediction_horizon = 10;
    params_.control_horizon = 5;
    params_.dt = 0.1;
    params_.wheel_base = 2.8;
    params_.max_steer = 0.5236;
    params_.min_steer = -0.5236;
    params_.max_steer_rate = 0.3;
    params_.min_steer_rate = -0.3;
    params_.max_acceleration = 2.0;
    params_.min_acceleration = -3.0;
    params_.max_velocity = 30.0;
    params_.min_velocity = 0.0;

    params_.Q(0, 0) = params_.Q_cte;
    params_.Q(1, 1) = params_.Q_heading;
    params_.Q(2, 2) = params_.Q_velocity;

    params_.R(0, 0) = params_.R_steer;
    params_.R(1, 1) = params_.R_accel;

    last_steer_ = 0.0;
    last_accel_ = 0.0;
    return true;
}

Eigen::VectorXd MPCController::predictState(const Eigen::VectorXd& state, double steer, double accel) {
    Eigen::VectorXd next_state(4);
    double x = state(0);
    double y = state(1);
    double yaw = state(2);
    double v = state(3);

    double limited_steer = std::max(params_.min_steer, std::min(params_.max_steer, steer));
    double limited_accel = std::max(params_.min_acceleration, std::min(params_.max_acceleration, accel));

    double new_v = v + limited_accel * params_.dt;
    new_v = std::max(params_.min_velocity, std::min(params_.max_velocity, new_v));

    double yaw_rate = (new_v / params_.wheel_base) * tan(limited_steer);
    double new_yaw = yaw + yaw_rate * params_.dt;

    while (new_yaw > M_PI) new_yaw -= 2 * M_PI;
    while (new_yaw < -M_PI) new_yaw += 2 * M_PI;

    double avg_v = (v + new_v) / 2.0;
    next_state(0) = x + avg_v * cos(yaw) * params_.dt;
    next_state(1) = y + avg_v * sin(yaw) * params_.dt;
    next_state(2) = new_yaw;
    next_state(3) = new_v;

    return next_state;
}

double MPCController::calculateCrossTrackError(const std::vector<ReferencePoint>& trajectory,
                                              const Eigen::VectorXd& state, int idx) {
    if (idx < 0 || idx >= trajectory.size()) return 0.0;
    const auto& ref = trajectory[idx];
    double dx = ref.x - state(0);
    double dy = ref.y - state(1);
    double yaw = state(2);
    return dy * cos(yaw) - dx * sin(yaw);
}

double MPCController::calculateHeadingError(const std::vector<ReferencePoint>& trajectory,
                                           const Eigen::VectorXd& state, int idx) {
    if (idx < 0 || idx >= trajectory.size()) return 0.0;
    double heading_error = trajectory[idx].yaw - state(2);
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;
    return heading_error;
}

bool MPCController::updateReferenceTrajectory(const Path& path, const VehicleState& current_state) {
    if (path.points.empty()) return false;
    reference_trajectory_.clear();
    for (const auto& point : path.points) {
        ReferencePoint ref;
        ref.x = point.x;
        ref.y = point.y;
        ref.yaw = point.yaw;
        ref.velocity = point.velocity;
        ref.curvature = point.curvature;
        reference_trajectory_.push_back(ref);
    }
    return true;
}

bool MPCController::solveOptimization(const Eigen::VectorXd& initial_state,
                                      std::vector<double>& controls) {
    int n = params_.prediction_horizon;
    int m = 2;
    int num_vars = n * m;

    Eigen::VectorXd x(num_vars);
    x.setZero();

    double learning_rate = 0.01;
    int iterations = 50;
    double epsilon = 0.0001;

    for (int iter = 0; iter < iterations; ++iter) {
        Eigen::VectorXd gradient(num_vars);
        gradient.setZero();

        for (int i = 0; i < num_vars; ++i) {
            Eigen::VectorXd x_plus = x;
            x_plus(i) += epsilon;
            
            Eigen::VectorXd state = initial_state;
            double cost_plus = 0.0;
            double prev_steer = last_steer_;

            for (int j = 0; j < n; ++j) {
                double steer = x_plus(j*m);
                double accel = x_plus(j*m + 1);
                state = predictState(state, steer, accel);

                int ref_idx = std::min(j, (int)reference_trajectory_.size() - 1);
                double cte = calculateCrossTrackError(reference_trajectory_, state, ref_idx);
                double heading_error = calculateHeadingError(reference_trajectory_, state, ref_idx);
                double velocity_error = reference_trajectory_[ref_idx].velocity - state(3);

                double steer_rate = steer - prev_steer;
                prev_steer = steer;

                cost_plus += params_.Q(0, 0) * cte * cte;
                cost_plus += params_.Q(1, 1) * heading_error * heading_error;
                cost_plus += params_.Q(2, 2) * velocity_error * velocity_error;
                cost_plus += params_.R(0, 0) * steer * steer;
                cost_plus += params_.R(1, 1) * accel * accel;
            }

            Eigen::VectorXd x_minus = x;
            x_minus(i) -= epsilon;

            state = initial_state;
            double cost_minus = 0.0;
            prev_steer = last_steer_;

            for (int j = 0; j < n; ++j) {
                double steer = x_minus(j*m);
                double accel = x_minus(j*m + 1);
                state = predictState(state, steer, accel);

                int ref_idx = std::min(j, (int)reference_trajectory_.size() - 1);
                double cte = calculateCrossTrackError(reference_trajectory_, state, ref_idx);
                double heading_error = calculateHeadingError(reference_trajectory_, state, ref_idx);
                double velocity_error = reference_trajectory_[ref_idx].velocity - state(3);

                double steer_rate = steer - prev_steer;
                prev_steer = steer;

                cost_minus += params_.Q(0, 0) * cte * cte;
                cost_minus += params_.Q(1, 1) * heading_error * heading_error;
                cost_minus += params_.Q(2, 2) * velocity_error * velocity_error;
                cost_minus += params_.R(0, 0) * steer * steer;
                cost_minus += params_.R(1, 1) * accel * accel;
            }

            gradient(i) = (cost_plus - cost_minus) / (2 * epsilon);
        }

        x -= learning_rate * gradient;

        for (int i = 0; i < n; ++i) {
            x(i*m) = std::max(params_.min_steer, std::min(params_.max_steer, x(i*m)));
            x(i*m + 1) = std::max(params_.min_acceleration, std::min(params_.max_acceleration, x(i*m + 1)));
        }
    }

    controls.resize(num_vars);
    for (int i = 0; i < num_vars; ++i) {
        controls[i] = x(i);
    }

    return true;
}

double MPCController::compute(const Path& path, const VehicleState& current_state) {
    if (path.points.empty()) return 0.0;

    updateReferenceTrajectory(path, current_state);

    Eigen::VectorXd initial_state(4);
    initial_state(0) = current_state.x;
    initial_state(1) = current_state.y;
    initial_state(2) = current_state.yaw;
    initial_state(3) = sqrt(current_state.vx * current_state.vx + current_state.vy * current_state.vy);

    std::vector<double> controls;
    if (!solveOptimization(initial_state, controls)) {
        return last_steer_;
    }

    double steer_cmd = controls[0];
    double accel_cmd = controls[1];

    double steer_rate = steer_cmd - last_steer_;
    if (std::abs(steer_rate) > params_.max_steer_rate * params_.dt) {
        steer_cmd = last_steer_ + std::copysign(params_.max_steer_rate * params_.dt, steer_rate);
    }

    last_steer_ = steer_cmd;
    last_accel_ = accel_cmd;

    return steer_cmd;
}

}
}