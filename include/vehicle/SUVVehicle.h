#ifndef SUV_VEHICLE_H
#define SUV_VEHICLE_H

#include "Vehicle.h"
#include "VehicleFactory.h"
#include "common/Config.h"

namespace AD {
namespace Vehicle {

class SUVVehicle : public Vehicle {
public:
    explicit SUVVehicle(const std::string& config_path);
    
    bool init(const std::string& config_path) override;
    void updateState(const VehicleState& state) override;
    VehicleState getState() const override;
    
    void sendControl(double throttle, double brake, double steer) override;
    
    std::string getModelName() const override { return "SUV"; }
    const VehicleParams& getParams() const override { return params_; }

private:
    void loadConfig(const std::string& config_path);
    void setupCommunication();
    
    std::string ethernet_ip_;
    int ethernet_port_;
};

}
}

#endif