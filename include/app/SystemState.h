#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <string>

namespace AD {
namespace App {

enum class SystemState {
    INIT,
    IDLE,
    PLANNING,
    TRACKING,
    OBSTACLE_AVOIDANCE,
    EMERGENCY_STOP,
    PARKING
};

enum class SystemEvent {
    EVENT_START,
    EVENT_GOAL_SET,
    EVENT_PATH_READY,
    EVENT_OBSTACLE_DETECTED,
    EVENT_COLLISION_WARNING,
    EVENT_GOAL_REACHED,
    EVENT_EMERGENCY,
    EVENT_PARK,
    EVENT_ERROR,
    EVENT_TIMEOUT,
    EVENT_RESUME
};

inline std::string stateToString(SystemState state) {
    switch (state) {
        case SystemState::INIT: return "INIT";
        case SystemState::IDLE: return "IDLE";
        case SystemState::PLANNING: return "PLANNING";
        case SystemState::TRACKING: return "TRACKING";
        case SystemState::OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOIDANCE";
        case SystemState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case SystemState::PARKING: return "PARKING";
        default: return "UNKNOWN";
    }
}

inline std::string eventToString(SystemEvent event) {
    switch (event) {
        case SystemEvent::EVENT_START: return "EVENT_START";
        case SystemEvent::EVENT_GOAL_SET: return "EVENT_GOAL_SET";
        case SystemEvent::EVENT_PATH_READY: return "EVENT_PATH_READY";
        case SystemEvent::EVENT_OBSTACLE_DETECTED: return "EVENT_OBSTACLE_DETECTED";
        case SystemEvent::EVENT_COLLISION_WARNING: return "EVENT_COLLISION_WARNING";
        case SystemEvent::EVENT_GOAL_REACHED: return "EVENT_GOAL_REACHED";
        case SystemEvent::EVENT_EMERGENCY: return "EVENT_EMERGENCY";
        case SystemEvent::EVENT_PARK: return "EVENT_PARK";
        case SystemEvent::EVENT_ERROR: return "EVENT_ERROR";
        case SystemEvent::EVENT_TIMEOUT: return "EVENT_TIMEOUT";
        case SystemEvent::EVENT_RESUME: return "EVENT_RESUME";
        default: return "UNKNOWN_EVENT";
    }
}

}
}

#endif