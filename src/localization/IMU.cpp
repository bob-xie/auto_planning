#include "localization/IMU.h"
#include <thread>
#include <chrono>

namespace AD {
namespace Localization {

bool IMU::init(const std::string& port) {
    return true;
}

void IMU::start() {
    running_ = true;
}

void IMU::stop() {
    running_ = false;
}

IMUData IMU::getLatestData() {
    return latest_data_;
}

void IMU::readLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

}
}
