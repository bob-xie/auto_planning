#ifndef SMOOTHER_MANAGER_H
#define SMOOTHER_MANAGER_H

#include "planning/smooth/SmootherFactory.h"
#include "common/Path.h"
#include <memory>
#include <string>

namespace AD {
namespace Planning {
namespace Smooth {

class SmootherManager {
public:
    SmootherManager();
    
    bool init(const std::string& config_file = "");
    
    bool initFromConfig(const std::string& smoother_type, 
                        double smoothing_factor = 0.5,
                        double resolution = 0.2,
                        double learning_rate = 0.1,
                        double weight_data = 0.5,
                        double weight_smooth = 0.5);
    
    Path smooth(const Path& input_path);
    
    std::string getCurrentSmootherName() const;
    
    bool isEnabled() const;
    
    void setEnabled(bool enabled);

private:
    std::unique_ptr<PathSmoother> smoother_;
    bool enabled_;
    std::string current_smoother_name_;
};

}
}
}

#endif