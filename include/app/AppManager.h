#ifndef APP_MANAGER_H
#define APP_MANAGER_H

#include <memory>
#include <thread>
#include <mutex>
#include "StateMachine.h"
#include "perception/CameraDriver.h"
#include "perception/LiDARDriver.h"
#include "perception/PerceptionBase.h"
#include "localization/FusionLocalizer.h"
#include "planning/GlobalPlanner.h"
#include "planning/LocalPlanner.h"
#include "planning/smooth/SmootherManager.h"
#include "control/longitudinal/LongitudinalController.h"
#include "control/ControllerBase.h"
#include "vehicle/Vehicle.h"
#include "safety/SafetyManager.h"
#include "common/VehicleState.h"
#include "common/Path.h"

namespace AD {
namespace App {

class AppManager {
public:
    AppManager();
    ~AppManager();
    
    bool init(const std::string& vehicle_model = "Sedan", const std::string& config_path = "");
    
    void run();
    
    void stop();
    
    void setGoal(double x, double y);
    
    StateMachine& getStateMachine() { return state_machine_; }
    
private:
    void updateLoop();
    void selectionLoop();
    
    void registerStateActions();
    
    void onEnterInit();
    void onEnterIdle();
    void onEnterPlanning();
    void onEnterTracking();
    void onEnterObstacleAvoidance();
    void onEnterEmergencyStop();
    void onEnterParking();
    
    void onExitInit();
    void onExitIdle();
    void onExitPlanning();
    void onExitTracking();
    void onExitObstacleAvoidance();
    void onExitEmergencyStop();
    void onExitParking();
    
    void onDoInit();
    void onDoIdle();
    void onDoPlanning();
    void onDoTracking();
    void onDoObstacleAvoidance();
    void onDoEmergencyStop();
    void onDoParking();
    
    StateMachine state_machine_;
    
    Perception::CameraDriver camera_;
    Perception::LiDARDriver lidar_;
    std::unique_ptr<Perception::PerceptionBase> detector_;
    
    Localization::FusionLocalizer localizer_;
    
    Planning::GlobalPlanner global_planner_;
    Planning::LocalPlanner local_planner_;
    Planning::Smooth::SmootherManager smoother_manager_;
    
    Control::LongitudinalController lon_controller_;
    std::unique_ptr<Control::ControllerBase> lat_controller_;
    
    std::unique_ptr<Vehicle::Vehicle> vehicle_;
    
    VehicleState current_state_;
    Path reference_path_;
    double target_speed_ = 10.0;
    
    bool running_ = false;
    std::thread update_thread_;
    std::thread selection_thread_;
    std::thread camera_thread_;
    std::thread lidar_thread_;
    
    std::mutex data_mutex_;
};

}
}

#endif