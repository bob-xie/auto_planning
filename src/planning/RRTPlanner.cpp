#include "planning/RRTPlanner.h"

namespace AD {
namespace Planning {

bool RRTPlanner::init(double grid_size, double obstacle_radius) {
    grid_size_ = grid_size;
    obstacle_radius_ = obstacle_radius;
    return true;
}

void RRTPlanner::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
}

RRTNode* RRTPlanner::getNearestNode(double x, double y) {
    RRTNode* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    
    for (RRTNode* node : nodes_) {
        double dist = sqrt(pow(x - node->x, 2) + pow(y - node->y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    return nearest;
}

bool RRTPlanner::isPathCollision(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = sqrt(dx * dx + dy * dy);
    
    int steps = static_cast<int>(dist / 0.5);
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = x1 + dx * t;
        double y = y1 + dy * t;
        
        for (const auto& obs : obstacles_) {
            double obs_dist = sqrt(pow(x - obs.x, 2) + pow(y - obs.y, 2));
            if (obs_dist < obs.radius + obstacle_radius_) {
                return true;
            }
        }
    }
    return false;
}

void RRTPlanner::clearNodes() {
    for (RRTNode* node : nodes_) {
        delete node;
    }
    nodes_.clear();
}

Path RRTPlanner::reconstructPath(RRTNode* goal_node) {
    Path path;
    RRTNode* current = goal_node;
    
    std::vector<std::pair<double, double>> points;
    while (current) {
        points.emplace_back(current->x, current->y);
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

Path RRTPlanner::plan(const VehicleState& start, const VehicleState& goal) {
    clearNodes();
    
    RRTNode* start_node = new RRTNode(start.x, start.y);
    nodes_.push_back(start_node);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(-50, 150);
    std::uniform_real_distribution<double> y_dist(-50, 150);
    
    for (int i = 0; i < max_iterations_; ++i) {
        double rand_x = x_dist(gen);
        double rand_y = y_dist(gen);
        
        if (i % 10 == 0) {
            rand_x = goal.x;
            rand_y = goal.y;
        }
        
        RRTNode* nearest = getNearestNode(rand_x, rand_y);
        
        double dx = rand_x - nearest->x;
        double dy = rand_y - nearest->y;
        double dist = sqrt(dx * dx + dy * dy);
        
        double step_size = std::min(max_step_, dist);
        double new_x = nearest->x + (dx / dist) * step_size;
        double new_y = nearest->y + (dy / dist) * step_size;
        
        if (isPathCollision(nearest->x, nearest->y, new_x, new_y)) {
            continue;
        }
        
        RRTNode* new_node = new RRTNode(new_x, new_y);
        new_node->parent = nearest;
        nodes_.push_back(new_node);
        
        double goal_dist = sqrt(pow(new_x - goal.x, 2) + pow(new_y - goal.y, 2));
        if (goal_dist < goal_tolerance_) {
            RRTNode* goal_node = new RRTNode(goal.x, goal.y);
            goal_node->parent = new_node;
            nodes_.push_back(goal_node);
            return reconstructPath(goal_node);
        }
    }
    
    return Path();
}

bool RRTStarPlanner::init(double grid_size, double obstacle_radius) {
    RRTPlanner::init(grid_size, obstacle_radius);
    return true;
}

void RRTStarPlanner::rewire(RRTNode* new_node) {
    for (RRTNode* node : nodes_) {
        if (node == new_node) continue;
        
        double dist = sqrt(pow(new_node->x - node->x, 2) + 
                          pow(new_node->y - node->y, 2));
        
        if (dist > rewire_radius_) continue;
        
        if (!isPathCollision(node->x, node->y, new_node->x, new_node->y)) {
            double potential_cost = 0;
            RRTNode* temp = node;
            while (temp) {
                if (temp->parent) {
                    double dx = temp->x - temp->parent->x;
                    double dy = temp->y - temp->parent->y;
                    potential_cost += sqrt(dx * dx + dy * dy);
                }
                temp = temp->parent;
            }
            potential_cost += dist;
            
            double current_cost = 0;
            temp = new_node;
            while (temp) {
                if (temp->parent) {
                    double dx = temp->x - temp->parent->x;
                    double dy = temp->y - temp->parent->y;
                    current_cost += sqrt(dx * dx + dy * dy);
                }
                temp = temp->parent;
            }
            
            if (potential_cost < current_cost) {
                new_node->parent = node;
            }
        }
    }
}

Path RRTStarPlanner::plan(const VehicleState& start, const VehicleState& goal) {
    clearNodes();
    
    RRTNode* start_node = new RRTNode(start.x, start.y);
    nodes_.push_back(start_node);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(-50, 150);
    std::uniform_real_distribution<double> y_dist(-50, 150);
    
    for (int i = 0; i < max_iterations_; ++i) {
        double rand_x = x_dist(gen);
        double rand_y = y_dist(gen);
        
        if (i % 10 == 0) {
            rand_x = goal.x;
            rand_y = goal.y;
        }
        
        RRTNode* nearest = getNearestNode(rand_x, rand_y);
        
        double dx = rand_x - nearest->x;
        double dy = rand_y - nearest->y;
        double dist = sqrt(dx * dx + dy * dy);
        
        double step_size = std::min(max_step_, dist);
        double new_x = nearest->x + (dx / dist) * step_size;
        double new_y = nearest->y + (dy / dist) * step_size;
        
        if (isPathCollision(nearest->x, nearest->y, new_x, new_y)) {
            continue;
        }
        
        RRTNode* new_node = new RRTNode(new_x, new_y);
        new_node->parent = nearest;
        nodes_.push_back(new_node);
        
        rewire(new_node);
        
        double goal_dist = sqrt(pow(new_x - goal.x, 2) + pow(new_y - goal.y, 2));
        if (goal_dist < goal_tolerance_) {
            RRTNode* goal_node = new RRTNode(goal.x, goal.y);
            goal_node->parent = new_node;
            nodes_.push_back(goal_node);
            return reconstructPath(goal_node);
        }
    }
    
    return Path();
}

}
}