#include "app/AppManager.h"
#include "perception/PerceptionFactory.h"
#include "vehicle/VehicleFactory.h"
#include "control/ControllerBase.h"
#include "common/Config.h"
#include <iostream>
#include <chrono>
#include <mutex>
#include <filesystem>
#include <algorithm>

namespace AD {
namespace App {

AppManager::AppManager() {
}

AppManager::~AppManager() {
    stop();
}

bool AppManager::init(const std::string& vehicle_model, const std::string& config_path) {
    std::filesystem::path base_path = std::filesystem::current_path();
    std::string config_file;
    
    if (config_path.empty()) {
        std::string lower_model = vehicle_model;
        std::transform(lower_model.begin(), lower_model.end(), lower_model.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        config_file = (base_path.parent_path() / "config" / "vehicles" / (lower_model + ".yaml")).string();
    } else {
        config_file = config_path;
    }
    
    Config config;
    if (!config.load(config_file)) {
        std::cerr << "[AppManager] Failed to load config: " << config_file << std::endl;
        return false;
    }
    
    auto& vehicle_factory = Vehicle::VehicleFactory::getInstance();
    vehicle_ = vehicle_factory.createVehicle(vehicle_model, config_file);
    if (!vehicle_) {
        std::cerr << "[AppManager] Failed to create vehicle: " << vehicle_model << std::endl;
        return false;
    }
    
    std::cout << "[AppManager] Vehicle loaded: " << vehicle_->getModelName() << std::endl;
    
    std::string controller_type = config.get<std::string>("controller.lateral.type", "Stanley");
    if (controller_type == "MPC") {
        lat_controller_ = Control::createController(Control::ControllerType::MPC);
        std::cout << "[AppManager] Lateral controller: MPC" << std::endl;
    } else {
        lat_controller_ = Control::createController(Control::ControllerType::STANLEY);
        std::cout << "[AppManager] Lateral controller: Stanley" << std::endl;
    }
    
    std::string detector_type = config.get<std::string>("perception.detector.type", "YOLO");
    auto& perception_factory = Perception::PerceptionFactory::getInstance();
    detector_ = perception_factory.createPerception(detector_type, config_file);
    if (detector_) {
        std::cout << "[AppManager] Detector: " << detector_->getName() << std::endl;
    }
    
    Control::LongitudinalControllerParams lon_params;
    lon_params.kp = config.get<double>("controller.longitudinal.kp", 1.0);
    lon_params.ki = config.get<double>("controller.longitudinal.ki", 0.1);
    lon_params.kd = config.get<double>("controller.longitudinal.kd", 0.05);
    lon_controller_.init(lon_params);
    lat_controller_->init();
    
    std::vector<Planning::Obstacle> obstacles;
    obstacles.emplace_back(30.0, 20.0, 2.0);
    obstacles.emplace_back(60.0, 35.0, 1.5);
    obstacles.emplace_back(80.0, 45.0, 2.0);
    global_planner_.setObstacles(obstacles);
    
    bool smoother_enabled = config.get<bool>("planner.smoother.enabled", true);
    std::string smoother_type = config.get<std::string>("planner.smoother.type", "BSPLINE");
    double smoothing_factor = config.get<double>("planner.smoother.parameters.smoothing_factor", 0.5);
    double resolution = config.get<double>("planner.smoother.parameters.resolution", 0.2);
    double learning_rate = config.get<double>("planner.smoother.parameters.learning_rate", 0.1);
    double weight_data = config.get<double>("planner.smoother.parameters.weight_data", 0.5);
    double weight_smooth = config.get<double>("planner.smoother.parameters.weight_smooth", 0.5);
    
    smoother_manager_.setEnabled(smoother_enabled);
    if (smoother_enabled) {
        smoother_manager_.initFromConfig(smoother_type, smoothing_factor, resolution,
                                        learning_rate, weight_data, weight_smooth);
        std::cout << "[AppManager] 路径平滑算法: " << smoother_manager_.getCurrentSmootherName() << std::endl;
    } else {
        std::cout << "[AppManager] 路径平滑已禁用" << std::endl;
    }
    
    registerStateActions();
    state_machine_.initialize();
    
    return true;
}

void AppManager::registerStateActions() {
    state_machine_.registerEnterAction(SystemState::INIT, std::bind(&AppManager::onEnterInit, this));
    state_machine_.registerEnterAction(SystemState::IDLE, std::bind(&AppManager::onEnterIdle, this));
    state_machine_.registerEnterAction(SystemState::PLANNING, std::bind(&AppManager::onEnterPlanning, this));
    state_machine_.registerEnterAction(SystemState::TRACKING, std::bind(&AppManager::onEnterTracking, this));
    state_machine_.registerEnterAction(SystemState::OBSTACLE_AVOIDANCE, std::bind(&AppManager::onEnterObstacleAvoidance, this));
    state_machine_.registerEnterAction(SystemState::EMERGENCY_STOP, std::bind(&AppManager::onEnterEmergencyStop, this));
    state_machine_.registerEnterAction(SystemState::PARKING, std::bind(&AppManager::onEnterParking, this));
    
    state_machine_.registerExitAction(SystemState::INIT, std::bind(&AppManager::onExitInit, this));
    state_machine_.registerExitAction(SystemState::IDLE, std::bind(&AppManager::onExitIdle, this));
    state_machine_.registerExitAction(SystemState::PLANNING, std::bind(&AppManager::onExitPlanning, this));
    state_machine_.registerExitAction(SystemState::TRACKING, std::bind(&AppManager::onExitTracking, this));
    state_machine_.registerExitAction(SystemState::OBSTACLE_AVOIDANCE, std::bind(&AppManager::onExitObstacleAvoidance, this));
    state_machine_.registerExitAction(SystemState::EMERGENCY_STOP, std::bind(&AppManager::onExitEmergencyStop, this));
    state_machine_.registerExitAction(SystemState::PARKING, std::bind(&AppManager::onExitParking, this));
    
    state_machine_.registerDoAction(SystemState::INIT, std::bind(&AppManager::onDoInit, this));
    state_machine_.registerDoAction(SystemState::IDLE, std::bind(&AppManager::onDoIdle, this));
    state_machine_.registerDoAction(SystemState::PLANNING, std::bind(&AppManager::onDoPlanning, this));
    state_machine_.registerDoAction(SystemState::TRACKING, std::bind(&AppManager::onDoTracking, this));
    state_machine_.registerDoAction(SystemState::OBSTACLE_AVOIDANCE, std::bind(&AppManager::onDoObstacleAvoidance, this));
    state_machine_.registerDoAction(SystemState::EMERGENCY_STOP, std::bind(&AppManager::onDoEmergencyStop, this));
    state_machine_.registerDoAction(SystemState::PARKING, std::bind(&AppManager::onDoParking, this));
}

void AppManager::run() {
    running_ = true;
    
    camera_thread_ = std::thread([this]() {
        camera_.init();
        camera_.start();
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        camera_.stop();
    });
    
    lidar_thread_ = std::thread([this]() {
        lidar_.init();
        lidar_.start();
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        lidar_.stop();
    });
    
    update_thread_ = std::thread([this]() {
        updateLoop();
    });
    
    selection_thread_ = std::thread([this]() {
        selectionLoop();
    });
    
    state_machine_.handleEvent(SystemEvent::EVENT_START);
}

void AppManager::stop() {
    running_ = false;
    
    if (camera_thread_.joinable()) {
        camera_thread_.join();
    }
    if (lidar_thread_.joinable()) {
        lidar_thread_.join();
    }
    if (update_thread_.joinable()) {
        update_thread_.join();
    }
    if (selection_thread_.joinable()) {
        selection_thread_.join();
    }
}

void AppManager::setGoal(double x, double y) {
    global_planner_.setGoal(x, y);
    state_machine_.setGoal(x, y);
    state_machine_.handleEvent(SystemEvent::EVENT_GOAL_SET);
}

void AppManager::updateLoop() {
    while (running_) {
        state_machine_.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void AppManager::selectionLoop() {
    while (running_) {
        std::cout << "\n\n=== 系统控制菜单 ===" << std::endl;
        std::cout << "当前状态: " << stateToString(state_machine_.getCurrentState()) << std::endl;
        std::cout << "\n操作选项:" << std::endl;
        std::cout << "1. 设置目标点 (100, 50)" << std::endl;
        std::cout << "2. 模拟检测障碍物" << std::endl;
        std::cout << "3. 模拟碰撞警告" << std::endl;
        std::cout << "4. 模拟到达目标" << std::endl;
        std::cout << "5. 停车" << std::endl;
        std::cout << "6. 紧急制动" << std::endl;
        std::cout << "7. 恢复运行" << std::endl;
        std::cout << "\n算法选择:" << std::endl;
        std::cout << "8. 全局算法: A*" << std::endl;
        std::cout << "9. 全局算法: RRT" << std::endl;
        std::cout << "10. 全局算法: RRT*" << std::endl;
        std::cout << "11. 局部算法: DWA" << std::endl;
        std::cout << "12. 局部算法: Lattice" << std::endl;
        std::cout << "\n请输入选择: ";
        
        int choice;
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                setGoal(100.0, 50.0);
                std::cout << "目标已设置: (100, 50)" << std::endl;
                break;
            case 2:
                state_machine_.setObstacleDetected(true);
                state_machine_.handleEvent(SystemEvent::EVENT_OBSTACLE_DETECTED);
                break;
            case 3:
                state_machine_.setCollisionWarning(true);
                state_machine_.handleEvent(SystemEvent::EVENT_COLLISION_WARNING);
                break;
            case 4:
                state_machine_.setGoalReached(true);
                state_machine_.handleEvent(SystemEvent::EVENT_GOAL_REACHED);
                break;
            case 5:
                state_machine_.handleEvent(SystemEvent::EVENT_PARK);
                break;
            case 6:
                state_machine_.handleEvent(SystemEvent::EVENT_EMERGENCY);
                break;
            case 7:
                state_machine_.handleEvent(SystemEvent::EVENT_RESUME);
                break;
            case 8:
                global_planner_.setPlannerType(Planning::PlannerType::ASTAR);
                std::cout << "全局算法已切换: A*" << std::endl;
                break;
            case 9:
                global_planner_.setPlannerType(Planning::PlannerType::RRT);
                std::cout << "全局算法已切换: RRT" << std::endl;
                break;
            case 10:
                global_planner_.setPlannerType(Planning::PlannerType::RRT_STAR);
                std::cout << "全局算法已切换: RRT*" << std::endl;
                break;
            case 11:
                local_planner_.setPlannerType(Planning::LocalPlannerType::DWA);
                std::cout << "局部算法已切换: DWA" << std::endl;
                break;
            case 12:
                local_planner_.setPlannerType(Planning::LocalPlannerType::LATTICE);
                std::cout << "局部算法已切换: Lattice" << std::endl;
                break;
            default:
                std::cout << "无效选择" << std::endl;
                break;
        }
    }
}

void AppManager::onEnterInit() {
    std::cout << "[AppManager] Entering INIT state" << std::endl;
}

void AppManager::onEnterIdle() {
    std::cout << "[AppManager] Entering IDLE state - Ready to accept commands" << std::endl;
    vehicle_->sendControl(0.0, 0.0, 0.0);
}

void AppManager::onEnterPlanning() {
    std::cout << "[AppManager] Entering PLANNING state - Computing path" << std::endl;
}

void AppManager::onEnterTracking() {
    std::cout << "[AppManager] Entering TRACKING state - Following path" << std::endl;
    target_speed_ = 10.0;
}

void AppManager::onEnterObstacleAvoidance() {
    std::cout << "[AppManager] Entering OBSTACLE_AVOIDANCE state - Replanning" << std::endl;
}

void AppManager::onEnterEmergencyStop() {
    std::cout << "[AppManager] Entering EMERGENCY_STOP state - Emergency braking" << std::endl;
    vehicle_->sendControl(0.0, 1.0, 0.0);
}

void AppManager::onEnterParking() {
    std::cout << "[AppManager] Entering PARKING state" << std::endl;
    vehicle_->sendControl(0.0, 0.5, 0.0);
}

void AppManager::onExitInit() {
    std::cout << "[AppManager] Exiting INIT state" << std::endl;
}

void AppManager::onExitIdle() {
    std::cout << "[AppManager] Exiting IDLE state" << std::endl;
}

void AppManager::onExitPlanning() {
    std::cout << "[AppManager] Exiting PLANNING state" << std::endl;
}

void AppManager::onExitTracking() {
    std::cout << "[AppManager] Exiting TRACKING state" << std::endl;
}

void AppManager::onExitObstacleAvoidance() {
    std::cout << "[AppManager] Exiting OBSTACLE_AVOIDANCE state" << std::endl;
}

void AppManager::onExitEmergencyStop() {
    std::cout << "[AppManager] Exiting EMERGENCY_STOP state" << std::endl;
}

void AppManager::onExitParking() {
    std::cout << "[AppManager] Exiting PARKING state" << std::endl;
}

void AppManager::onDoInit() {
}

void AppManager::onDoIdle() {
}

void AppManager::onDoPlanning() {
    auto [goal_x, goal_y] = state_machine_.getGoal();
    
    if (goal_x != 0.0 || goal_y != 0.0) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        reference_path_ = global_planner_.plan(current_state_);
        
        if (!reference_path_.points.empty()) {
            if (smoother_manager_.isEnabled()) {
                reference_path_ = smoother_manager_.smooth(reference_path_);
                std::cout << "[AppManager] 路径已平滑处理" << std::endl;
            }
            state_machine_.setCurrentPath(reference_path_);
            state_machine_.handleEvent(SystemEvent::EVENT_PATH_READY);
        }
    }
}

void AppManager::onDoTracking() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto camera_data = camera_.getLatestData();
    auto lidar_data = lidar_.getLatestData();
    
    localizer_.update(camera_data, lidar_data);
    current_state_ = localizer_.getState();
    state_machine_.setVehicleState(current_state_);
    
    vehicle_->updateState(current_state_);
    
    if (detector_) {
        auto detected_objects = detector_->detect(camera_data, lidar_data);
        if (!detected_objects.empty()) {
            std::vector<Planning::Obstacle> obstacles;
            for (const auto& obj : detected_objects) {
                obstacles.emplace_back(obj.x, obj.y, obj.width / 2.0);
            }
            global_planner_.setObstacles(obstacles);
            state_machine_.setObstacleDetected(true);
        }
    }
    
    auto obstacle_info = local_planner_.detectObstacles(lidar_data);
    reference_path_ = local_planner_.replan(reference_path_, obstacle_info);
    
    auto& safety_manager = Safety::SafetyManager::getInstance();
    double pos_cov = localizer_.getPositionCovariance();
    safety_manager.update(current_state_, pos_cov);
    
    if (!safety_manager.isSafe()) {
        state_machine_.handleEvent(SystemEvent::EVENT_EMERGENCY);
        return;
    }
    
    target_speed_ = std::min(target_speed_, vehicle_->getParams().max_speed);
    auto [throttle, brake] = lon_controller_.compute(target_speed_, current_state_);
    double steering = lat_controller_->compute(reference_path_, current_state_);
    
    vehicle_->sendControl(throttle, brake, steering);
    
    std::cout << "\r位置: (" << current_state_.x << ", " << current_state_.y << ")"
              << " | 速度: " << current_state_.vx
              << " | 油门: " << throttle
              << " | 刹车: " << brake
              << " | 转向: " << steering
              << std::flush;
}

void AppManager::onDoObstacleAvoidance() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto lidar_data = lidar_.getLatestData();
    auto obstacle_info = local_planner_.detectObstacles(lidar_data);
    
    reference_path_ = local_planner_.replan(reference_path_, obstacle_info);
    
    if (!reference_path_.points.empty()) {
        state_machine_.setCurrentPath(reference_path_);
        state_machine_.handleEvent(SystemEvent::EVENT_PATH_READY);
    }
}

void AppManager::onDoEmergencyStop() {
    vehicle_->sendControl(0.0, 1.0, 0.0);
}

void AppManager::onDoParking() {
    vehicle_->sendControl(0.0, 0.3, 0.0);
}

}
}