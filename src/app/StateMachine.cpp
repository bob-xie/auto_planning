#include "app/StateMachine.h"
#include <iostream>

namespace AD {
namespace App {

StateMachine::StateMachine() : current_state_(SystemState::INIT) {
}

void StateMachine::initialize() {
    std::cout << "[StateMachine] Initializing state machine" << std::endl;
    
    if (enter_actions_.count(SystemState::INIT)) {
        enter_actions_[SystemState::INIT]();
    }
}

SystemState StateMachine::transition(SystemState current, SystemEvent event) {
    switch (current) {
        case SystemState::INIT: {
            if (event == SystemEvent::EVENT_START) {
                return SystemState::IDLE;
            }
            break;
        }
        case SystemState::IDLE: {
            if (event == SystemEvent::EVENT_GOAL_SET) {
                return SystemState::PLANNING;
            } else if (event == SystemEvent::EVENT_PARK) {
                return SystemState::PARKING;
            } else if (event == SystemEvent::EVENT_EMERGENCY) {
                return SystemState::EMERGENCY_STOP;
            }
            break;
        }
        case SystemState::PLANNING: {
            if (event == SystemEvent::EVENT_PATH_READY) {
                return SystemState::TRACKING;
            } else if (event == SystemEvent::EVENT_EMERGENCY) {
                return SystemState::EMERGENCY_STOP;
            } else if (event == SystemEvent::EVENT_ERROR) {
                return SystemState::IDLE;
            }
            break;
        }
        case SystemState::TRACKING: {
            if (event == SystemEvent::EVENT_OBSTACLE_DETECTED) {
                return SystemState::OBSTACLE_AVOIDANCE;
            } else if (event == SystemEvent::EVENT_COLLISION_WARNING) {
                return SystemState::EMERGENCY_STOP;
            } else if (event == SystemEvent::EVENT_GOAL_REACHED) {
                return SystemState::PARKING;
            } else if (event == SystemEvent::EVENT_EMERGENCY) {
                return SystemState::EMERGENCY_STOP;
            } else if (event == SystemEvent::EVENT_ERROR) {
                return SystemState::IDLE;
            }
            break;
        }
        case SystemState::OBSTACLE_AVOIDANCE: {
            if (event == SystemEvent::EVENT_PATH_READY) {
                return SystemState::TRACKING;
            } else if (event == SystemEvent::EVENT_COLLISION_WARNING) {
                return SystemState::EMERGENCY_STOP;
            } else if (event == SystemEvent::EVENT_EMERGENCY) {
                return SystemState::EMERGENCY_STOP;
            } else if (event == SystemEvent::EVENT_TIMEOUT) {
                return SystemState::EMERGENCY_STOP;
            }
            break;
        }
        case SystemState::EMERGENCY_STOP: {
            if (event == SystemEvent::EVENT_RESUME) {
                return SystemState::IDLE;
            }
            break;
        }
        case SystemState::PARKING: {
            if (event == SystemEvent::EVENT_START) {
                return SystemState::IDLE;
            } else if (event == SystemEvent::EVENT_EMERGENCY) {
                return SystemState::EMERGENCY_STOP;
            }
            break;
        }
        default:
            break;
    }
    return current;
}

bool StateMachine::handleEvent(SystemEvent event) {
    SystemState next_state = transition(current_state_, event);
    
    if (next_state != current_state_) {
        std::cout << "[StateMachine] Transition: " 
                  << stateToString(current_state_) 
                  << " -> " 
                  << stateToString(next_state)
                  << " (Event: " << eventToString(event) << ")" << std::endl;
        
        if (exit_actions_.count(current_state_)) {
            exit_actions_[current_state_]();
        }
        
        current_state_ = next_state;
        
        if (enter_actions_.count(current_state_)) {
            enter_actions_[current_state_]();
        }
        
        return true;
    }
    
    return false;
}

void StateMachine::registerEnterAction(SystemState state, StateAction action) {
    enter_actions_[state] = action;
}

void StateMachine::registerExitAction(SystemState state, StateAction action) {
    exit_actions_[state] = action;
}

void StateMachine::registerDoAction(SystemState state, StateAction action) {
    do_actions_[state] = action;
}

void StateMachine::update() {
    if (do_actions_.count(current_state_)) {
        do_actions_[current_state_]();
    }
}

}
}