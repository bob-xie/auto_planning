#ifndef PERCEPTION_BASE_H
#define PERCEPTION_BASE_H

#include <vector>
#include <string>
#include <memory>
#include "common/VehicleState.h"
#include "CameraDriver.h"
#include "LiDARDriver.h"

namespace AD {
namespace Perception {

struct DetectedObject {
    enum class Type {
        CAR,
        PEDESTRIAN,
        CYCLIST,
        OBSTACLE
    };
    
    Type type;
    double x;
    double y;
    double z;
    double width;
    double length;
    double height;
    double velocity_x;
    double velocity_y;
    double confidence;
    std::string id;
};

using DetectedObjects = std::vector<DetectedObject>;

class PerceptionBase {
public:
    virtual ~PerceptionBase() = default;
    
    virtual bool init(const std::string& config_path) = 0;
    virtual DetectedObjects detect(const CameraData& camera_data, 
                                   const LiDARData& lidar_data) = 0;
    virtual std::string getName() const = 0;
    
protected:
    bool initialized_ = false;
};

using PerceptionPtr = std::unique_ptr<PerceptionBase>;

}
}

#endif