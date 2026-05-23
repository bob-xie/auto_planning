#include "vehicle/SedanVehicle.h"

namespace AD {
namespace Vehicle {

SedanVehicle::SedanVehicle(const std::string& config_path) {
    init(config_path);
}

bool SedanVehicle::init(const std::string& config_path) {
    loadConfig(config_path);
    setupCommunication();
    return true;
}

void SedanVehicle::loadConfig(const std::string& config_path) {
    Config config;
    if (config.load(config_path)) {
        params_.wheelbase = config.get<double>("vehicle.wheelbase", 2.8);
        params_.max_steer = config.get<double>("vehicle.max_steer", 0.5236);
        params_.max_speed = config.get<double>("vehicle.max_speed", 120.0 / 3.6);
        params_.max_accel = config.get<double>("vehicle.max_accel", 2.0);
        params_.max_decel = config.get<double>("vehicle.max_decel", 4.0);
        params_.mass = config.get<double>("vehicle.mass", 1500.0);
        params_.front_stiffness = config.get<double>("vehicle.front_stiffness", 150000.0);
        params_.rear_stiffness = config.get<double>("vehicle.rear_stiffness", 150000.0);
        
        can_interface_ = config.get<std::string>("communication.can_interface", "can0");
        can_bitrate_ = config.get<int>("communication.can_bitrate", 500000);
    }
}

void SedanVehicle::setupCommunication() {
}

void SedanVehicle::updateState(const VehicleState& state) {
    state_ = state;
}

VehicleState SedanVehicle::getState() const {
    return state_;
}

void SedanVehicle::sendControl(double throttle, double brake, double steer) {
    throttle = std::max(0.0, std::min(1.0, throttle));
    brake = std::max(0.0, std::min(1.0, brake));
    steer = std::max(-params_.max_steer, std::min(params_.max_steer, steer));
}

REGISTER_VEHICLE("Sedan", SedanVehicle);

}
}