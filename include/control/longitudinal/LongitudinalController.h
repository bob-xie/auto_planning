#ifndef LONGITUDINAL_CONTROLLER_H
#define LONGITUDINAL_CONTROLLER_H

#include "common/VehicleState.h"

namespace AD {
namespace Control {

struct LongitudinalControllerParams {
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.05;
    
    double throttle_max = 1.0;
    double throttle_min = 0.0;
    double brake_max = 1.0;
    double brake_min = 0.0;
    
    double integral_max = 5.0;
};

class LongitudinalController {
public:
    LongitudinalController() = default;
    
    void init(const LongitudinalControllerParams& params);
    
    void reset();
    
    std::pair<double, double> compute(double target_speed, 
                                       const VehicleState& current_state);

private:
    LongitudinalControllerParams params_;
    
    double integral_error_ = 0.0;
    double last_error_ = 0.0;
};

}
}

#endif