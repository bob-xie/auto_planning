#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <memory>
#include "common/VehicleState.h"

namespace AD {
namespace Vehicle {

struct VehicleParams {
    double wheelbase = 2.8;
    double max_steer = 0.5236;
    double max_speed = 120.0 / 3.6;
    double max_accel = 2.0;
    double max_decel = 4.0;
    double mass = 1500.0;
    double front_stiffness = 150000.0;
    double rear_stiffness = 150000.0;
};

class Vehicle {
public:
    virtual ~Vehicle() = default;
    
    virtual bool init(const std::string& config_path) = 0;
    virtual void updateState(const VehicleState& state) = 0;
    virtual VehicleState getState() const = 0;
    
    virtual void sendControl(double throttle, double brake, double steer) = 0;
    
    virtual std::string getModelName() const = 0;
    virtual const VehicleParams& getParams() const = 0;
    
protected:
    VehicleParams params_;
    VehicleState state_;
};

using VehiclePtr = std::unique_ptr<Vehicle>;

}
}

#endif