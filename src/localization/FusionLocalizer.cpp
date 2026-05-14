#include "localization/FusionLocalizer.h"

namespace AD {
namespace Localization {

bool FusionLocalizer::init() {
    gps_.init();
    imu_.init();
    
    current_state_.x = 0.0;
    current_state_.y = 0.0;
    current_state_.yaw = 0.0;
    current_state_.vx = 0.0;
    current_state_.wz = 0.0;
    
    ekf_.init(current_state_);
    last_update_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
    
    return true;
}

void FusionLocalizer::update(const Perception::CameraData& camera_data, 
                             const Perception::LiDARData& lidar_data) {
    if (!initialized_) {
        init();
        return;
    }
    
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_update_time_).count();
    last_update_time_ = current_time;
    
    if (dt > 0.0) {
        ekf_.predict(dt, 0.0, 0.0);
    }
    
    GPSData gps_data = gps_.getLatestData();
    if (gps_data.fix_type >= 2) {
        ekf_.updateGPS(gps_data);
    }
    
    IMUData imu_data = imu_.getLatestData();
    ekf_.updateIMU(imu_data);
    
    current_state_ = ekf_.getState();
}

VehicleState FusionLocalizer::getState() const {
    return current_state_;
}

double FusionLocalizer::getPositionCovariance() const {
    Eigen::MatrixXd P = ekf_.getCovariance();
    return P(0, 0) + P(1, 1);
}

}
}