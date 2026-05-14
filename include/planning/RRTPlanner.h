#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include "PathPlanner.h"
#include <vector>
#include <random>

namespace AD {
namespace Planning {

struct RRTNode {
    double x;
    double y;
    RRTNode* parent;
    
    RRTNode(double x_, double y_) 
        : x(x_), y(y_), parent(nullptr) {}
};

class RRTPlanner : public PathPlanner {
public:
    bool init(double grid_size = 1.0, double obstacle_radius = 0.5) override;
    
    Path plan(const VehicleState& start, const VehicleState& goal) override;
    
    void setObstacles(const std::vector<Obstacle>& obstacles) override;
    
    std::string getName() const override { return "RRT"; }

protected:
    double grid_size_;
    double obstacle_radius_;
    std::vector<Obstacle> obstacles_;
    std::vector<RRTNode*> nodes_;
    
    double max_step_ = 2.0;
    double goal_tolerance_ = 1.0;
    int max_iterations_ = 500;
    
    RRTNode* getNearestNode(double x, double y);
    
    bool isPathCollision(double x1, double y1, double x2, double y2);
    
    Path reconstructPath(RRTNode* goal_node);
    
    void clearNodes();
};

class RRTStarPlanner : public RRTPlanner {
public:
    bool init(double grid_size = 1.0, double obstacle_radius = 0.5) override;
    
    Path plan(const VehicleState& start, const VehicleState& goal) override;
    
    std::string getName() const override { return "RRT*"; }

private:
    double rewire_radius_ = 5.0;
    
    void rewire(RRTNode* new_node);
};

}
}

#endif