#ifndef VEHICLE_FACTORY_H
#define VEHICLE_FACTORY_H

#include <string>
#include <memory>
#include <unordered_map>
#include <functional>
#include "Vehicle.h"

namespace AD {
namespace Vehicle {

class VehicleFactory {
public:
    using VehicleCreator = std::function<VehiclePtr(const std::string&)>;
    
    static VehicleFactory& getInstance() {
        static VehicleFactory instance;
        return instance;
    }
    
    bool registerVehicle(const std::string& model_name, VehicleCreator creator) {
        creators_[model_name] = creator;
        return true;
    }
    
    VehiclePtr createVehicle(const std::string& model_name, 
                            const std::string& config_path) const {
        auto it = creators_.find(model_name);
        if (it != creators_.end()) {
            return it->second(config_path);
        }
        return nullptr;
    }
    
    std::vector<std::string> getSupportedVehicles() const {
        std::vector<std::string> models;
        for (const auto& pair : creators_) {
            models.push_back(pair.first);
        }
        return models;
    }

private:
    VehicleFactory() = default;
    ~VehicleFactory() = default;
    
    VehicleFactory(const VehicleFactory&) = delete;
    VehicleFactory& operator=(const VehicleFactory&) = delete;
    
    std::unordered_map<std::string, VehicleCreator> creators_;
};

template<typename T>
class VehicleRegistrar {
public:
    VehicleRegistrar(const std::string& model_name) {
        VehicleFactory::getInstance().registerVehicle(model_name, 
            [](const std::string& config_path) -> VehiclePtr {
                return std::make_unique<T>(config_path);
            });
    }
};

#define REGISTER_VEHICLE(model_name, class_name) \
    static AD::Vehicle::VehicleRegistrar<class_name> vehicle_registrar_##class_name(model_name)

}
}

#endif