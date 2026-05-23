#include "communication/CommunicationManager.h"
#include "common/Config.h"
#include <chrono>

namespace AD {
namespace Communication {

bool CommunicationManager::init(const std::string& config_path) {
    return true;
}

#ifdef BUILD_WITH_PROTOBUF

bool CommunicationManager::sendVehicleState(const AD::VehicleState& state) {
    MessageFrame frame;
    frame.set_type(MessageFrame::VEHICLE_STATE);
    
    VehicleState* proto_state = frame.mutable_vehicle_state();
    *proto_state = convertToProtoVehicleState(state);
    
    return true;
}

bool CommunicationManager::sendPath(const AD::Path& path) {
    MessageFrame frame;
    frame.set_type(MessageFrame::PATH);
    
    Path* proto_path = frame.mutable_path();
    *proto_path = convertToProtoPath(path);
    
    return true;
}

bool CommunicationManager::sendObstacles(const std::vector<AD::Perception::DetectedObject>& obstacles) {
    MessageFrame frame;
    frame.set_type(MessageFrame::OBSTACLES);
    
    for (const auto& obj : obstacles) {
        Obstacle* proto_obstacle = frame.add_obstacles();
        *proto_obstacle = convertToProtoObstacle(obj);
    }
    
    return true;
}

bool CommunicationManager::sendControlCommand(double throttle, double brake, double steering, double target_speed) {
    MessageFrame frame;
    frame.set_type(MessageFrame::CONTROL_COMMAND);
    
    ControlCommand* cmd = frame.mutable_control_command();
    cmd->set_throttle(throttle);
    cmd->set_brake(brake);
    cmd->set_steering(steering);
    cmd->set_target_speed(target_speed);
    cmd->set_timestamp(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
    
    return true;
}

bool CommunicationManager::sendSystemStatus(SystemStatus::Status status, const std::string& message) {
    MessageFrame frame;
    frame.set_type(MessageFrame::SYSTEM_STATUS);
    
    SystemStatus* sys_status = frame.mutable_system_status();
    sys_status->set_status(status);
    sys_status->set_message(message);
    sys_status->set_timestamp(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
    
    return true;
}

bool CommunicationManager::receiveMessage(const std::vector<uint8_t>& data, MessageFrame& frame) {
    return frame.ParseFromArray(data.data(), data.size());
}

AD::VehicleState CommunicationManager::parseVehicleState(const VehicleState& proto_state) {
    AD::VehicleState state;
    state.x = proto_state.x();
    state.y = proto_state.y();
    state.z = proto_state.z();
    state.roll = proto_state.roll();
    state.pitch = proto_state.pitch();
    state.yaw = proto_state.yaw();
    state.vx = proto_state.vx();
    state.vy = proto_state.vy();
    state.ax = proto_state.ax();
    state.wz = proto_state.wz();
    state.throttle = proto_state.throttle();
    state.brake = proto_state.brake();
    state.steering = proto_state.steering();
    state.timestamp = proto_state.timestamp();
    return state;
}

AD::Path CommunicationManager::parsePath(const Path& proto_path) {
    AD::Path path;
    for (const auto& proto_point : proto_path.points()) {
        path.addPoint(
            proto_point.x(),
            proto_point.y(),
            proto_point.z(),
            proto_point.yaw(),
            proto_point.velocity()
        );
    }
    return path;
}

std::vector<AD::Perception::DetectedObject> CommunicationManager::parseObstacles(
    const google::protobuf::RepeatedPtrField<Obstacle>& proto_obstacles) {
    
    std::vector<AD::Perception::DetectedObject> obstacles;
    for (const auto& proto_obj : proto_obstacles) {
        AD::Perception::DetectedObject obj;
        obj.id = proto_obj.id();
        obj.x = proto_obj.x();
        obj.y = proto_obj.y();
        obj.z = proto_obj.z();
        obj.width = proto_obj.width();
        obj.length = proto_obj.length();
        obj.height = proto_obj.height();
        obj.velocity_x = proto_obj.velocity_x();
        obj.velocity_y = proto_obj.velocity_y();
        obj.confidence = proto_obj.confidence();
        
        switch (proto_obj.type()) {
            case Obstacle::CAR:
                obj.type = AD::Perception::DetectedObject::Type::CAR;
                break;
            case Obstacle::PEDESTRIAN:
                obj.type = AD::Perception::DetectedObject::Type::PEDESTRIAN;
                break;
            case Obstacle::CYCLIST:
                obj.type = AD::Perception::DetectedObject::Type::CYCLIST;
                break;
            default:
                obj.type = AD::Perception::DetectedObject::Type::OBSTACLE;
                break;
        }
        
        obstacles.push_back(obj);
    }
    return obstacles;
}

void CommunicationManager::registerCallback(MessageCallback callback) {
    callback_ = callback;
}

std::vector<uint8_t> CommunicationManager::serializeMessage(const MessageFrame& frame) {
    std::vector<uint8_t> data(frame.ByteSizeLong());
    frame.SerializeToArray(data.data(), data.size());
    return data;
}

VehicleState CommunicationManager::convertToProtoVehicleState(const AD::VehicleState& state) {
    VehicleState proto_state;
    proto_state.set_x(state.x);
    proto_state.set_y(state.y);
    proto_state.set_z(state.z);
    proto_state.set_roll(state.roll);
    proto_state.set_pitch(state.pitch);
    proto_state.set_yaw(state.yaw);
    proto_state.set_vx(state.vx);
    proto_state.set_vy(state.vy);
    proto_state.set_ax(state.ax);
    proto_state.set_wz(state.wz);
    proto_state.set_throttle(state.throttle);
    proto_state.set_brake(state.brake);
    proto_state.set_steering(state.steering);
    proto_state.set_timestamp(state.timestamp);
    return proto_state;
}

Path CommunicationManager::convertToProtoPath(const AD::Path& path) {
    Path proto_path;
    for (const auto& point : path.points) {
        PathPoint* proto_point = proto_path.add_points();
        proto_point->set_x(point.x);
        proto_point->set_y(point.y);
        proto_point->set_z(point.z);
        proto_point->set_yaw(point.yaw);
        proto_point->set_velocity(point.velocity);
        proto_point->set_curvature(point.curvature);
        proto_point->set_s(point.s);
    }
    return proto_path;
}

Obstacle CommunicationManager::convertToProtoObstacle(const AD::Perception::DetectedObject& obj) {
    Obstacle proto_obj;
    proto_obj.set_id(obj.id);
    proto_obj.set_x(obj.x);
    proto_obj.set_y(obj.y);
    proto_obj.set_z(obj.z);
    proto_obj.set_width(obj.width);
    proto_obj.set_length(obj.length);
    proto_obj.set_height(obj.height);
    proto_obj.set_velocity_x(obj.velocity_x);
    proto_obj.set_velocity_y(obj.velocity_y);
    proto_obj.set_confidence(obj.confidence);
    
    switch (obj.type) {
        case AD::Perception::DetectedObject::Type::CAR:
            proto_obj.set_type(Obstacle::CAR);
            break;
        case AD::Perception::DetectedObject::Type::PEDESTRIAN:
            proto_obj.set_type(Obstacle::PEDESTRIAN);
            break;
        case AD::Perception::DetectedObject::Type::CYCLIST:
            proto_obj.set_type(Obstacle::CYCLIST);
            break;
        default:
            proto_obj.set_type(Obstacle::STATIC);
            break;
    }
    
    return proto_obj;
}

#else

bool CommunicationManager::sendVehicleState(const AD::VehicleState& state) {
    return true;
}

bool CommunicationManager::sendPath(const AD::Path& path) {
    return true;
}

bool CommunicationManager::sendObstacles(const std::vector<AD::Perception::DetectedObject>& obstacles) {
    return true;
}

bool CommunicationManager::sendControlCommand(double throttle, double brake, double steering, double target_speed) {
    return true;
}

#endif

}
}