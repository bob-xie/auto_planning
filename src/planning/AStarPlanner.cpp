#include "planning/AStarPlanner.h"
#include <algorithm>

namespace AD {
namespace Planning {

bool AStarPlanner::init(double grid_size, double obstacle_radius) {
    grid_size_ = grid_size;
    obstacle_radius_ = obstacle_radius;
    vehicle_length_ = 4.0;
    vehicle_width_ = 1.8;
    safety_distance_ = 0.5;
    return true;
}

void AStarPlanner::setVehicleDimensions(double length, double width) {
    vehicle_length_ = length;
    vehicle_width_ = width;
}

void AStarPlanner::setSafetyDistance(double distance) {
    safety_distance_ = distance;
}

void AStarPlanner::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool AStarPlanner::isCollision(int x, int y, double yaw) {
    double world_x = x * grid_size_;
    double world_y = y * grid_size_;
    
    double half_length = vehicle_length_ / 2.0 + safety_distance_;
    double half_width = vehicle_width_ / 2.0 + safety_distance_;
    
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    
    std::vector<std::pair<double, double>> corners = {
        {world_x + half_length * cos_yaw - half_width * sin_yaw,
         world_y + half_length * sin_yaw + half_width * cos_yaw},
        {world_x + half_length * cos_yaw + half_width * sin_yaw,
         world_y + half_length * sin_yaw - half_width * cos_yaw},
        {world_x - half_length * cos_yaw - half_width * sin_yaw,
         world_y - half_length * sin_yaw + half_width * cos_yaw},
        {world_x - half_length * cos_yaw + half_width * sin_yaw,
         world_y - half_length * sin_yaw - half_width * cos_yaw}
    };
    
    for (const auto& obs : obstacles_) {
        for (const auto& corner : corners) {
            double dist = sqrt(pow(corner.first - obs.x, 2) + pow(corner.second - obs.y, 2));
            if (dist < obs.radius + obstacle_radius_) {
                return true;
            }
        }
        
        double dist_center = sqrt(pow(world_x - obs.x, 2) + pow(world_y - obs.y, 2));
        if (dist_center < obs.radius + half_length + obstacle_radius_) {
            double closest_x = world_x + (obs.x - world_x) * half_length / dist_center;
            double closest_y = world_y + (obs.y - world_y) * half_length / dist_center;
            
            double dx = obs.x - closest_x;
            double dy = obs.y - closest_y;
            double dist_side = sqrt(dx * dx + dy * dy);
            
            if (dist_side < obs.radius + half_width + obstacle_radius_) {
                return true;
            }
        }
    }
    return false;
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y) {
    std::vector<std::pair<int, int>> neighbors = {
        {x+1, y}, {x-1, y}, {x, y+1}, {x, y-1},
        {x+1, y+1}, {x+1, y-1}, {x-1, y+1}, {x-1, y-1}
    };
    return neighbors;
}

Path AStarPlanner::plan(const VehicleState& start, const VehicleState& goal) {
    Path path;
    
    int start_x = static_cast<int>(start.x / grid_size_);
    int start_y = static_cast<int>(start.y / grid_size_);
    int goal_x = static_cast<int>(goal.x / grid_size_);
    int goal_y = static_cast<int>(goal.y / grid_size_);
    
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, 
                        std::function<bool(AStarNode*, AStarNode*)>> 
        open_list([](AStarNode* a, AStarNode* b) { return a->f > b->f; });
    
    std::unordered_map<int, std::unordered_map<int, AStarNode*>> all_nodes;
    
    AStarNode* start_node = new AStarNode(start_x, start_y);
    start_node->g = 0;
    start_node->h = heuristic(start_x, start_y, goal_x, goal_y);
    start_node->f = start_node->g + start_node->h;
    open_list.push(start_node);
    all_nodes[start_x][start_y] = start_node;
    
    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    
    while (!open_list.empty()) {
        AStarNode* current = open_list.top();
        open_list.pop();
        
        if (current->x == goal_x && current->y == goal_y) {
            return reconstructPath(current);
        }
        
        for (const auto& dir : directions) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;
            
            if (isCollision(nx, ny)) continue;
            
            double cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0;
            double new_g = current->g + cost;
            
            if (!all_nodes[nx][ny]) {
                AStarNode* neighbor = new AStarNode(nx, ny);
                neighbor->g = new_g;
                neighbor->h = heuristic(nx, ny, goal_x, goal_y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                open_list.push(neighbor);
                all_nodes[nx][ny] = neighbor;
            } else if (new_g < all_nodes[nx][ny]->g) {
                all_nodes[nx][ny]->g = new_g;
                all_nodes[nx][ny]->f = new_g + all_nodes[nx][ny]->h;
                all_nodes[nx][ny]->parent = current;
            }
        }
    }
    
    return path;
}

Path AStarPlanner::reconstructPath(AStarNode* goal_node) {
    Path path;
    AStarNode* current = goal_node;
    
    std::vector<std::pair<double, double>> points;
    while (current) {
        points.emplace_back(current->x * grid_size_, current->y * grid_size_);
        current = current->parent;
    }
    
    std::reverse(points.begin(), points.end());
    
    for (size_t i = 0; i < points.size(); ++i) {
        double yaw = 0.0;
        if (i < points.size() - 1) {
            yaw = atan2(points[i+1].second - points[i].second, 
                        points[i+1].first - points[i].first);
        }
        path.addPoint(points[i].first, points[i].second, 0.0, yaw, 10.0);
    }
    
    return path;
}

}
}