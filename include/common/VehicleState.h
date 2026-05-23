#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <Eigen/Dense>

namespace AD {

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    
    double vx = 0.0;
    double vy = 0.0;
    double ax = 0.0;
    double wz = 0.0;
    
    double throttle = 0.0;
    double brake = 0.0;
    double steering = 0.0;
    
    double timestamp = 0.0;
    
    Eigen::Vector3d getPosition() const {
        return Eigen::Vector3d(x, y, z);
    }
    
    Eigen::Quaterniond getQuaternion() const {
        return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }
};

}

#endif