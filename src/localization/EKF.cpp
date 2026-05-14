#include "localization/EKF.h"
#include <cmath>

namespace AD {
namespace Localization {

EKF::EKF() {
    state_.resize(6);
    state_.setZero();
    
    P_.resize(6, 6);
    P_.setIdentity();
    P_ *= 0.1;
    
    F_.resize(6, 6);
    F_.setIdentity();
    
    Q_.resize(6, 6);
    Q_.setIdentity();
    Q_ *= 0.01;
    
    H_gps_.resize(3, 6);
    H_gps_.setZero();
    H_gps_(0, 0) = 1.0;
    H_gps_(1, 1) = 1.0;
    H_gps_(2, 3) = 1.0;
    
    R_gps_.resize(3, 3);
    R_gps_.setIdentity();
    R_gps_ *= 0.5;
    
    H_imu_.resize(3, 6);
    H_imu_.setZero();
    H_imu_(0, 2) = 1.0;
    H_imu_(1, 4) = 1.0;
    H_imu_(2, 5) = 1.0;
    
    R_imu_.resize(3, 3);
    R_imu_.setIdentity();
    R_imu_ *= 0.01;
}

void EKF::init(const VehicleState& initial_state) {
    state_(0) = initial_state.x;
    state_(1) = initial_state.y;
    state_(2) = initial_state.yaw;
    state_(3) = initial_state.vx;
    state_(4) = initial_state.ax;
    state_(5) = initial_state.wz;
    
    P_.setIdentity();
    P_ *= 0.1;
}

void EKF::predict(double dt, double steering, double acceleration) {
    double x = state_(0);
    double y = state_(1);
    double yaw = state_(2);
    double v = state_(3);
    double a = acceleration;
    double wz = state_(5);
    
    double new_x = x + v * cos(yaw) * dt;
    double new_y = y + v * sin(yaw) * dt;
    double new_yaw = yaw + wz * dt;
    double new_v = v + a * dt;
    double new_a = a;
    double new_wz = (v / wheelbase_) * tan(steering);
    
    state_(0) = new_x;
    state_(1) = new_y;
    state_(2) = new_yaw;
    state_(3) = new_v;
    state_(4) = new_a;
    state_(5) = new_wz;
    
    F_(0, 2) = -v * sin(yaw) * dt;
    F_(0, 3) = cos(yaw) * dt;
    F_(1, 2) = v * cos(yaw) * dt;
    F_(1, 3) = sin(yaw) * dt;
    F_(2, 5) = dt;
    F_(3, 4) = dt;
    
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::updateGPS(const GPSData& gps_data) {
    Eigen::VectorXd z(3);
    z(0) = gps_data.lon;
    z(1) = gps_data.lat;
    z(2) = gps_data.heading;
    
    update(z, H_gps_, R_gps_);
}

void EKF::updateIMU(const IMUData& imu_data) {
    Eigen::VectorXd z(3);
    z(0) = imu_data.orientation(2);
    z(1) = imu_data.linear_acceleration(0);
    z(2) = imu_data.angular_velocity(2);
    
    update(z, H_imu_, R_imu_);
}

void EKF::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    Eigen::VectorXd y = z - H * state_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    state_ = state_ + K * y;
    
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    P_ = (I - K * H) * P_;
}

VehicleState EKF::getState() const {
    VehicleState state;
    state.x = state_(0);
    state.y = state_(1);
    state.yaw = state_(2);
    state.vx = state_(3);
    state.ax = state_(4);
    state.wz = state_(5);
    return state;
}

Eigen::MatrixXd EKF::getCovariance() const {
    return P_;
}

}
}