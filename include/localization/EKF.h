#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include "common/VehicleState.h"
#include "RTKGPS.h"
#include "IMU.h"

namespace AD {
namespace Localization {

class EKF {
public:
    EKF();
    
    void init(const VehicleState& initial_state);
    
    void predict(double dt, double steering, double acceleration);
    
    void updateGPS(const GPSData& gps_data);
    
    void updateIMU(const IMUData& imu_data);
    
    VehicleState getState() const;
    
    Eigen::MatrixXd getCovariance() const;

private:
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_gps_;
    Eigen::MatrixXd R_gps_;
    Eigen::MatrixXd H_imu_;
    Eigen::MatrixXd R_imu_;
    
    double wheelbase_ = 2.8;
    
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);
};

}
}

#endif