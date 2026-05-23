#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "common/Path.h"
#include "common/VehicleState.h"
#include "PathPlanner.h"
#include <memory>

namespace AD {
namespace Planning {

class GlobalPlanner {
public:
    bool init();
    
    bool setPlannerType(PlannerType type);
    
    Path plan(const VehicleState& start);
    
    bool needUpdate() const;
    
    void setGoal(double x, double y);
    
    void setObstacles(const std::vector<Obstacle>& obstacles);
    
    std::string getCurrentPlannerName() const;

private:
    Path global_path_;
    VehicleState goal_;
    bool need_update_ = true;
    std::unique_ptr<PathPlanner> planner_;
    
    Path generateStraightPath(const VehicleState& start, const VehicleState& end);
};

}
}

#endif