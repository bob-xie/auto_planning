#include "control/longitudinal/LongitudinalController.h"

namespace AD {
namespace Control {

void LongitudinalController::init(const LongitudinalControllerParams& params) {
    params_ = params;
}

void LongitudinalController::reset() {
    integral_error_ = 0.0;
    last_error_ = 0.0;
}

std::pair<double, double> LongitudinalController::compute(double target_speed, 
                                                           const VehicleState& current_state) {
    double error = target_speed - current_state.vx;
    
    integral_error_ += error;
    integral_error_ = std::max(-params_.integral_max, 
                               std::min(params_.integral_max, integral_error_));
    
    double derivative = error - last_error_;
    last_error_ = error;
    
    double output = params_.kp * error + 
                    params_.ki * integral_error_ + 
                    params_.kd * derivative;
    
    double throttle = 0.0;
    double brake = 0.0;
    
    if (output > 0) {
        throttle = std::min(params_.throttle_max, std::max(params_.throttle_min, output));
    } else {
        brake = std::min(params_.brake_max, std::max(params_.brake_min, -output));
    }
    
    return {throttle, brake};
}

}
}