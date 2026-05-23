#include "planning/smooth/SmootherManager.h"
#include "common/Config.h"
#include <iostream>

namespace AD {
namespace Planning {
namespace Smooth {

SmootherManager::SmootherManager() : enabled_(true) {}

bool SmootherManager::init(const std::string& config_file) {
    Config config;
    
    if (!config_file.empty() && !config.load(config_file)) {
        std::cerr << "[SmootherManager] 无法加载配置文件: " << config_file << std::endl;
        return false;
    }
    
    enabled_ = config.get<bool>("planning.smoother.enabled", true);
    
    if (!enabled_) {
        std::cout << "[SmootherManager] 路径平滑已禁用" << std::endl;
        return true;
    }
    
    std::string smoother_type = config.get<std::string>("planning.smoother.type", "BSPLINE");
    double smoothing_factor = config.get<double>("planning.smoother.parameters.smoothing_factor", 0.5);
    double resolution = config.get<double>("planning.smoother.parameters.resolution", 0.2);
    double learning_rate = config.get<double>("planning.smoother.parameters.learning_rate", 0.1);
    double weight_data = config.get<double>("planning.smoother.parameters.weight_data", 0.5);
    double weight_smooth = config.get<double>("planning.smoother.parameters.weight_smooth", 0.5);
    
    return initFromConfig(smoother_type, smoothing_factor, resolution, 
                          learning_rate, weight_data, weight_smooth);
}

bool SmootherManager::initFromConfig(const std::string& smoother_type,
                                     double smoothing_factor,
                                     double resolution,
                                     double learning_rate,
                                     double weight_data,
                                     double weight_smooth) {
    SmootherType type = SmootherFactory::stringToType(smoother_type);
    smoother_ = SmootherFactory::create(type);
    
    if (!smoother_) {
        std::cerr << "[SmootherManager] 无法创建平滑器: " << smoother_type << std::endl;
        return false;
    }
    
    smoother_->init(smoothing_factor, resolution);
    current_smoother_name_ = smoother_->getName();
    
    std::cout << "[SmootherManager] 初始化完成" << std::endl;
    std::cout << "[SmootherManager] 算法: " << current_smoother_name_ << std::endl;
    std::cout << "[SmootherManager] 参数: smoothing_factor=" << smoothing_factor 
              << ", resolution=" << resolution << std::endl;
    
    return true;
}

Path SmootherManager::smooth(const Path& input_path) {
    if (!enabled_ || !smoother_) {
        return input_path;
    }
    
    return smoother_->smooth(input_path);
}

std::string SmootherManager::getCurrentSmootherName() const {
    return current_smoother_name_;
}

bool SmootherManager::isEnabled() const {
    return enabled_;
}

void SmootherManager::setEnabled(bool enabled) {
    enabled_ = enabled;
}

}
}
}