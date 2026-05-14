#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <memory>
#include <string>
#include <functional>
#include "common/VehicleState.h"
#include "common/Path.h"
#include "perception/PerceptionBase.h"

#ifdef BUILD_WITH_PROTOBUF
#include "communication.pb.h"
#endif

namespace AD {
namespace Communication {

class CommunicationManager {
public:
#ifdef BUILD_WITH_PROTOBUF
    using MessageCallback = std::function<void(const MessageFrame&)>;
#endif
    
    CommunicationManager() = default;
    ~CommunicationManager() = default;
    
    bool init(const std::string& config_path = "");
    
    bool sendVehicleState(const AD::VehicleState& state);
    bool sendPath(const AD::Path& path);
    bool sendObstacles(const std::vector<AD::Perception::DetectedObject>& obstacles);
    bool sendControlCommand(double throttle, double brake, double steering, double target_speed);
    
#ifdef BUILD_WITH_PROTOBUF
    bool sendSystemStatus(AD::Communication::SystemStatus::Status status, const std::string& message);
    
    bool receiveMessage(const std::vector<uint8_t>& data, MessageFrame& frame);
    
    AD::VehicleState parseVehicleState(const VehicleState& proto_state);
    AD::Path parsePath(const Path& proto_path);
    std::vector<AD::Perception::DetectedObject> parseObstacles(const google::protobuf::RepeatedPtrField<Obstacle>& proto_obstacles);
    
    void registerCallback(MessageCallback callback);
#endif
    
private:
#ifdef BUILD_WITH_PROTOBUF
    MessageCallback callback_ = nullptr;
    
    std::vector<uint8_t> serializeMessage(const MessageFrame& frame);
    
    VehicleState convertToProtoVehicleState(const AD::VehicleState& state);
    Path convertToProtoPath(const AD::Path& path);
    Obstacle convertToProtoObstacle(const AD::Perception::DetectedObject& obj);
#endif
};

}
}

#endif