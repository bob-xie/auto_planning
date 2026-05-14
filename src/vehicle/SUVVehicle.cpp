#include "vehicle/SUVVehicle.h"

namespace AD {
namespace Vehicle {

SUVVehicle::SUVVehicle(const std::string& config_path) {
    init(config_path);
}

bool SUVVehicle::init(const std::string& config_path) {
    loadConfig(config_path);
    setupCommunication();
    return true;
}

void SUVVehicle::loadConfig(const std::string& config_path) {
    Config config;
    if (config.load(config_path)) {
        params_.wheelbase = config.get<double>("vehicle.wheelbase", 3.0);
        params_.max_steer = config.get<double>("vehicle.max_steer", 0.4363);
        params_.max_speed = config.get<double>("vehicle.max_speed", 180.0 / 3.6);
        params_.max_accel = config.get<double>("vehicle.max_accel", 3.0);
        params_.max_decel = config.get<double>("vehicle.max_decel", 5.0);
        params_.mass = config.get<double>("vehicle.mass", 2000.0);
        params_.front_stiffness = config.get<double>("vehicle.front_stiffness", 200000.0);
        params_.rear_stiffness = config.get<double>("vehicle.rear_stiffness", 200000.0);
        
        ethernet_ip_ = config.get<std::string>("communication.ethernet_ip", "192.168.0.100");
        ethernet_port_ = config.get<int>("communication.ethernet_port", 8080);
    }
}

void SUVVehicle::setupCommunication() {
}

void SUVVehicle::updateState(const VehicleState& state) {
    state_ = state;
}

VehicleState SUVVehicle::getState() const {
    return state_;
}

void SUVVehicle::sendControl(double throttle, double brake, double steer) {
    throttle = std::max(0.0, std::min(1.0, throttle));
    brake = std::max(0.0, std::min(1.0, brake));
    steer = std::max(-params_.max_steer, std::min(params_.max_steer, steer));
}

REGISTER_VEHICLE("SUV", SUVVehicle);

}
}