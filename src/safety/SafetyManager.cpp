#include "safety/SafetyManager.h"
#include <iostream>

namespace AD {
namespace Safety {

SafetyManager& SafetyManager::getInstance() {
    static SafetyManager instance;
    return instance;
}

void SafetyManager::init() {
    safe_ = true;
    active_fault_ = FaultType::NONE;
    faults_.clear();
}

void SafetyManager::update(const VehicleState& state, double pos_covariance) {
    checkVehicleState(state);
    checkPositionUncertainty(pos_covariance);
    evaluateFaults();
}

void SafetyManager::checkVehicleState(const VehicleState& state) {
    const double MAX_SPEED = 50.0;
    const double MAX_ACCEL = 5.0;
    const double MAX_STEER_RATE = 1.0;
    
    if (std::abs(state.vx) > MAX_SPEED) {
        reportFault(FaultType::OVERSPEED, FaultLevel::ERROR, 
                   "车辆超速: " + std::to_string(state.vx) + " m/s");
    }
    
    if (std::abs(state.ax) > MAX_ACCEL) {
        reportFault(FaultType::CONTROL_FAULT, FaultLevel::WARNING,
                   "加速度超限: " + std::to_string(state.ax) + " m/s^2");
    }
}

void SafetyManager::checkPositionUncertainty(double pos_covariance) {
    const double MAX_COVARIANCE = 1.0;
    
    if (pos_covariance > MAX_COVARIANCE) {
        reportFault(FaultType::POSITION_UNCERTAINTY_HIGH, FaultLevel::ERROR,
                   "定位不确定性过高: " + std::to_string(pos_covariance));
    }
}

void SafetyManager::reportFault(FaultType type, FaultLevel level, const std::string& description) {
    auto it = faults_.find(type);
    
    if (it != faults_.end()) {
        it->second.active = true;
        it->second.description = description;
        it->second.timestamp = std::chrono::steady_clock::now();
    } else {
        FaultInfo info;
        info.type = type;
        info.level = level;
        info.description = description;
        info.timestamp = std::chrono::steady_clock::now();
        info.active = true;
        faults_[type] = info;
    }
    
    std::cout << "\n[安全告警] " << description << std::endl;
}

void SafetyManager::evaluateFaults() {
    active_fault_ = FaultType::NONE;
    safe_ = true;
    
    for (const auto& pair : faults_) {
        if (pair.second.active) {
            if (pair.second.level == FaultLevel::CRITICAL || 
                pair.second.level == FaultLevel::ERROR) {
                active_fault_ = pair.first;
                safe_ = false;
                
                if (pair.second.level == FaultLevel::CRITICAL) {
                    emergencyStop();
                }
                break;
            }
        }
    }
}

bool SafetyManager::isSafe() const {
    return safe_;
}

FaultType SafetyManager::getActiveFault() const {
    return active_fault_;
}

void SafetyManager::resetFault(FaultType type) {
    auto it = faults_.find(type);
    if (it != faults_.end()) {
        it->second.active = false;
    }
}

void SafetyManager::emergencyStop() {
    std::cout << "\n[紧急制动] 触发紧急停车!" << std::endl;
}

}
}