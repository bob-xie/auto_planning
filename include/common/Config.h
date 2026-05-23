#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <yaml-cpp/yaml.h>

namespace AD {

class Config {
public:
    bool load(const std::string& filename);
    
    template<typename T>
    T get(const std::string& key, const T& default_val = T()) const {
        try {
            return config_[key].as<T>();
        } catch (...) {
            return default_val;
        }
    }
    
private:
    YAML::Node config_;
};

}

#endif