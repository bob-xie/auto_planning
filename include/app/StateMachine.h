#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <functional>
#include <map>
#include <memory>
#include <string>
#include "SystemState.h"
#include "common/VehicleState.h"
#include "common/Path.h"

namespace AD {
namespace App {

class StateMachine {
public:
    using StateAction = std::function<void()>;
    using StateTransition = std::function<SystemState(SystemState, SystemEvent)>;
    
    StateMachine();
    
    void initialize();
    
    bool handleEvent(SystemEvent event);
    
    SystemState getCurrentState() const { return current_state_; }
    
    void setVehicleState(const VehicleState& state) { vehicle_state_ = state; }
    const VehicleState& getVehicleState() const { return vehicle_state_; }
    
    void setCurrentPath(const Path& path) { current_path_ = path; }
    const Path& getCurrentPath() const { return current_path_; }
    
    void setGoal(double x, double y) { goal_x_ = x; goal_y_ = y; }
    std::pair<double, double> getGoal() const { return {goal_x_, goal_y_}; }
    
    void setObstacleDetected(bool detected) { obstacle_detected_ = detected; }
    bool isObstacleDetected() const { return obstacle_detected_; }
    
    void setCollisionWarning(bool warning) { collision_warning_ = warning; }
    bool isCollisionWarning() const { return collision_warning_; }
    
    void setGoalReached(bool reached) { goal_reached_ = reached; }
    bool isGoalReached() const { return goal_reached_; }
    
    void registerEnterAction(SystemState state, StateAction action);
    void registerExitAction(SystemState state, StateAction action);
    void registerDoAction(SystemState state, StateAction action);
    
    void update();
    
private:
    SystemState current_state_;
    VehicleState vehicle_state_;
    Path current_path_;
    
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    
    bool obstacle_detected_ = false;
    bool collision_warning_ = false;
    bool goal_reached_ = false;
    
    std::map<SystemState, StateAction> enter_actions_;
    std::map<SystemState, StateAction> exit_actions_;
    std::map<SystemState, StateAction> do_actions_;
    
    SystemState transition(SystemState current, SystemEvent event);
};

}
}

#endif