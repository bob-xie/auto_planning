#include "planning/GlobalPlanner.h"
#include <cmath>

namespace AD {
namespace Planning {

bool GlobalPlanner::init() {
    planner_ = createPlanner(PlannerType::ASTAR);
    return planner_->init();
}

bool GlobalPlanner::setPlannerType(PlannerType type) {
    planner_ = createPlanner(type);
    bool result = planner_->init();
    need_update_ = true;
    return result;
}

Path GlobalPlanner::plan(const VehicleState& start) {
    if (!planner_) {
        return generateStraightPath(start, goal_);
    }
    
    Path path = planner_->plan(start, goal_);
    
    if (path.empty()) {
        return generateStraightPath(start, goal_);
    }
    
    need_update_ = false;
    return path;
}

bool GlobalPlanner::needUpdate() const {
    return need_update_;
}

void GlobalPlanner::setGoal(double x, double y) {
    goal_.x = x;
    goal_.y = y;
    need_update_ = true;
}

void GlobalPlanner::setObstacles(const std::vector<Obstacle>& obstacles) {
    if (planner_) {
        planner_->setObstacles(obstacles);
        need_update_ = true;
    }
}

std::string GlobalPlanner::getCurrentPlannerName() const {
    if (planner_) {
        return planner_->getName();
    }
    return "None";
}

Path GlobalPlanner::generateStraightPath(const VehicleState& start, 
                                         const VehicleState& end) {
    Path path;
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dist = sqrt(dx * dx + dy * dy);
    
    int num_points = static_cast<int>(dist / 0.5) + 1;
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        path.addPoint(start.x + dx * t, start.y + dy * t, 0.0, 
                      atan2(dy, dx), 10.0);
    }
    return path;
}

}
}