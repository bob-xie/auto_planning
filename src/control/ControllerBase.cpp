#include "control/ControllerBase.h"
#include "control/lateral/LateralController.h"
#include "control/mpc/MPCController.h"

namespace AD {
namespace Control {

std::unique_ptr<ControllerBase> createController(ControllerType type) {
    switch (type) {
        case ControllerType::STANLEY:
            return std::make_unique<LateralController>();
        case ControllerType::MPC:
            return std::make_unique<MPCController>();
        default:
            return std::make_unique<LateralController>();
    }
}

}
}