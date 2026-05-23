#include "planning/PathPlanner.h"
#include "planning/AStarPlanner.h"
#include "planning/RRTPlanner.h"

namespace AD {
namespace Planning {

std::unique_ptr<PathPlanner> createPlanner(PlannerType type) {
    switch (type) {
        case PlannerType::ASTAR:
            return std::make_unique<AStarPlanner>();
        case PlannerType::RRT:
            return std::make_unique<RRTPlanner>();
        case PlannerType::RRT_STAR:
            return std::make_unique<RRTStarPlanner>();
        default:
            return std::make_unique<AStarPlanner>();
    }
}

}
}