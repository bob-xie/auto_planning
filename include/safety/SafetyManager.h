#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <string>
#include <map>
#include <chrono>
#include <memory>
#include "common/VehicleState.h"

namespace AD {
namespace Safety {

enum class FaultLevel {
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

enum class FaultType {
    NONE,
    SENSOR_FAILURE,
    COMMUNICATION_LOSS,
    CONTROL_FAULT,
    POSITION_UNCERTAINTY_HIGH,
    EMERGENCY_BRAKE,
    OVERSPEED,
    STEERING_FAULT,
    POWER_FAULT
};

struct FaultInfo {
    FaultType type;
    FaultLevel level;
    std::string description;
    std::chrono::steady_clock::time_point timestamp;
    bool active;
};

class SafetyManager {
public:
    static SafetyManager& getInstance();
    
    void init();
    
    void update(const VehicleState& state, double pos_covariance);
    
    void reportFault(FaultType type, FaultLevel level, const std::string& description);
    
    bool isSafe() const;
    
    FaultType getActiveFault() const;
    
    void resetFault(FaultType type);
    
    void emergencyStop();
    
private:
    SafetyManager() = default;
    ~SafetyManager() = default;
    
    SafetyManager(const SafetyManager&) = delete;
    SafetyManager& operator=(const SafetyManager&) = delete;
    
    std::map<FaultType, FaultInfo> faults_;
    FaultType active_fault_ = FaultType::NONE;
    bool safe_ = true;
    
    void checkVehicleState(const VehicleState& state);
    void checkPositionUncertainty(double pos_covariance);
    void evaluateFaults();
};

}
}

#endif