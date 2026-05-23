#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <memory>
#include "common/Path.h"
#include "common/VehicleState.h"

namespace AD {
namespace Planning {

enum class PlannerType {
    ASTAR,
    RRT,
    RRT_STAR
};

struct Obstacle {
    double x;
    double y;
    double radius;
    
    Obstacle(double x_, double y_, double radius_) 
        : x(x_), y(y_), radius(radius_) {}
};

class PathPlanner {
public:
    virtual ~PathPlanner() = default;
    
    virtual bool init(double grid_size = 1.0, double obstacle_radius = 0.5) = 0;
    
    virtual Path plan(const VehicleState& start, const VehicleState& goal) = 0;
    
    virtual void setObstacles(const std::vector<Obstacle>& obstacles) = 0;
    
    virtual std::string getName() const = 0;
};

std::unique_ptr<PathPlanner> createPlanner(PlannerType type);

}
}

#endif