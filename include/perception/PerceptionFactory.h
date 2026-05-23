#ifndef PERCEPTION_FACTORY_H
#define PERCEPTION_FACTORY_H

#include <string>
#include <memory>
#include <unordered_map>
#include <functional>
#include "PerceptionBase.h"

namespace AD {
namespace Perception {

enum class DetectorType {
    YOLO,
    POINTPILLARS
};

class PerceptionFactory {
public:
    using PerceptionCreator = std::function<PerceptionPtr(const std::string&)>;
    
    static PerceptionFactory& getInstance() {
        static PerceptionFactory instance;
        return instance;
    }
    
    bool registerPerception(const std::string& name, PerceptionCreator creator) {
        creators_[name] = creator;
        return true;
    }
    
    PerceptionPtr createPerception(const std::string& name, 
                                   const std::string& config_path) const {
        auto it = creators_.find(name);
        if (it != creators_.end()) {
            return it->second(config_path);
        }
        return nullptr;
    }
    
    PerceptionPtr createPerception(DetectorType type, 
                                   const std::string& config_path) const {
        std::string name;
        switch (type) {
            case DetectorType::YOLO:
                name = "YOLO";
                break;
            case DetectorType::POINTPILLARS:
                name = "PointPillars";
                break;
            default:
                name = "YOLO";
        }
        return createPerception(name, config_path);
    }
    
    std::vector<std::string> getSupportedDetectors() const {
        std::vector<std::string> detectors;
        for (const auto& pair : creators_) {
            detectors.push_back(pair.first);
        }
        return detectors;
    }

private:
    PerceptionFactory() = default;
    ~PerceptionFactory() = default;
    
    PerceptionFactory(const PerceptionFactory&) = delete;
    PerceptionFactory& operator=(const PerceptionFactory&) = delete;
    
    std::unordered_map<std::string, PerceptionCreator> creators_;
};

template<typename T>
class PerceptionRegistrar {
public:
    PerceptionRegistrar(const std::string& name) {
        PerceptionFactory::getInstance().registerPerception(name,
            [](const std::string& config_path) -> PerceptionPtr {
                return std::make_unique<T>(config_path);
            });
    }
};

#define REGISTER_PERCEPTION(name, class_name) \
    static AD::Perception::PerceptionRegistrar<class_name> perception_registrar_##class_name(name)

}
}

#endif