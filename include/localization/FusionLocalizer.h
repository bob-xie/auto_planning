#ifndef FUSION_LOCALIZER_H
#define FUSION_LOCALIZER_H

#include "RTKGPS.h"
#include "IMU.h"
#include "EKF.h"
#include "common/VehicleState.h"
#include "perception/CameraDriver.h"
#include "perception/LiDARDriver.h"
#include <chrono>

namespace AD {
namespace Localization {

class FusionLocalizer {
public:
    bool init();
    void update(const Perception::CameraData& camera_data, 
                const Perception::LiDARData& lidar_data);
    
    VehicleState getState() const;
    double getPositionCovariance() const;

private:
    VehicleState current_state_;
    RTKGPS gps_;
    IMU imu_;
    EKF ekf_;
    
    std::chrono::steady_clock::time_point last_update_time_;
    bool initialized_ = false;
};

}
}

#endif