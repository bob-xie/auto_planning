#include "perception/LiDARDriver.h"
#include <thread>
#include <chrono>

namespace AD {
namespace Perception {

bool LiDARDriver::init(const std::string& ip, int port) {
    return true;
}

void LiDARDriver::start() {
    running_ = true;
}

void LiDARDriver::stop() {
    running_ = false;
}

LiDARData LiDARDriver::getLatestData() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_data_;
}

void LiDARDriver::receiveLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

}
}
