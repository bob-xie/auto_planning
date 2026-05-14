#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "PathPlanner.h"
#include <queue>
#include <unordered_map>
#include <cmath>

namespace AD {
namespace Planning {

struct AStarNode {
    int x;
    int y;
    double g;
    double h;
    double f;
    AStarNode* parent;
    
    AStarNode(int x_, int y_) 
        : x(x_), y(y_), g(0), h(0), f(0), parent(nullptr) {}
    
    bool operator>(const AStarNode& other) const {
        return f > other.f;
    }
};

class AStarPlanner : public PathPlanner {
public:
    bool init(double grid_size = 1.0, double obstacle_radius = 0.5) override;
    
    void setVehicleDimensions(double length, double width);
    
    void setSafetyDistance(double distance);
    
    Path plan(const VehicleState& start, const VehicleState& goal) override;
    
    void setObstacles(const std::vector<Obstacle>& obstacles) override;
    
    std::string getName() const override { return "A*"; }

private:
    double grid_size_;
    double obstacle_radius_;
    double vehicle_length_;
    double vehicle_width_;
    double safety_distance_;
    std::vector<Obstacle> obstacles_;
    
    double heuristic(int x1, int y1, int x2, int y2);
    
    bool isCollision(int x, int y, double yaw = 0.0);
    
    std::vector<std::pair<int, int>> getNeighbors(int x, int y);
    
    Path reconstructPath(AStarNode* goal_node);
};

}
}

#endif