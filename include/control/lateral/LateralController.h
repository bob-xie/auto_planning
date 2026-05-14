#ifndef LATERAL_CONTROLLER_H
#define LATERAL_CONTROLLER_H

#include "../ControllerBase.h"
#include "common/VehicleState.h"
#include "common/Path.h"

namespace AD {
namespace Control {

struct LateralControllerParams {
    double k = 0.5;
    double k_soft = 1.0;
    double max_steer = 0.5236;
    double min_steer = -0.5236;
};

class LateralController : public ControllerBase {
public:
    LateralController() = default;
    
    bool init() override;
    
    double compute(const Path& path, const VehicleState& current_state) override;
    
    std::string getName() const override { return "Stanley"; }

private:
    LateralControllerParams params_;
    
    double calculateCrossTrackError(const Path& path, 
                                     const VehicleState& state);
    
    double calculateTargetHeading(const Path& path, 
                                   const VehicleState& state);
};

}
}

#endif