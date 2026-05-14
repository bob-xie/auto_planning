#ifndef FOXGLOVE_VISUALIZER_H
#define FOXGLOVE_VISUALIZER_H

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include "common/Path.h"
#include "common/VehicleState.h"
#include "planning/PathPlanner.h"

namespace AD {
namespace FoxVisual {

class FoxgloveVisualizer {
public:
    FoxgloveVisualizer();
    ~FoxgloveVisualizer();

    bool init(const std::string& host = "0.0.0.0", int port = 8765);
    void start();
    void stop();

    void publishObstacles(const std::vector<Planning::Obstacle>& obstacles);
    void publishOriginalPath(const Path& path);
    void publishSmoothedPath(const Path& path);
    void publishVehicleState(const VehicleState& state);

private:
    void serverLoop();
    void encodeAndSendMessage(const std::string& topic, const std::string& data);

    std::string host_;
    int port_;
    std::atomic<bool> running_;
    std::thread server_thread_;
    
    std::mutex data_mutex_;
    std::vector<Planning::Obstacle> current_obstacles_;
    Path current_original_path_;
    Path current_smoothed_path_;
    VehicleState current_vehicle_state_;
};

}
}

#endif
