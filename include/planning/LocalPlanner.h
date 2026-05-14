#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "common/Path.h"
#include "common/VehicleState.h"
#include "perception/LiDARDriver.h"
#include "DWAPlanner.h"
#include "LatticePlanner.h"
#include <vector>

namespace AD {
namespace Planning {

enum class LocalPlannerType {
    DWA,
    LATTICE
};

class LocalPlanner {
public:
    bool init();
    
    void setPlannerType(LocalPlannerType type);
    
    LocalPlannerType getPlannerType() const;
    
    std::string getCurrentPlannerName() const;
    
    std::vector<ObstacleInfo> detectObstacles(const Perception::LiDARData& lidar_data);
    
    Path replan(const Path& original_path, 
                const std::vector<ObstacleInfo>& obstacles);
    
    double getTargetSpeed() const;

private:
    double target_speed_ = 0.0;
    bool obstacle_ahead_ = false;
    LocalPlannerType planner_type_ = LocalPlannerType::DWA;
    
    DWAPlanner dwa_planner_;
    LatticePlanner lattice_planner_;
    
    std::vector<LatticeObstacle> convertToLatticeObstacles(const std::vector<ObstacleInfo>& obstacles);
};

}
}

#endif