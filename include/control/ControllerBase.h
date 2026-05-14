#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <memory>
#include "common/VehicleState.h"
#include "common/Path.h"

namespace AD {
namespace Control {

enum class ControllerType {
    STANLEY,
    MPC
};

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    
    virtual bool init() = 0;
    
    virtual double compute(const Path& path, const VehicleState& current_state) = 0;
    
    virtual std::string getName() const = 0;
};

std::unique_ptr<ControllerBase> createController(ControllerType type);

}
}

#endif