#include "perception/CameraDriver.h"

namespace AD {
namespace Perception {

bool CameraDriver::init(const std::string& device) {
    cap_.open(device);
    return cap_.isOpened();
}

void CameraDriver::start() {
    running_ = true;
}

void CameraDriver::stop() {
    running_ = false;
}

CameraData CameraDriver::getLatestData() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_data_;
}

void CameraDriver::captureLoop() {
    while (running_) {
        cv::Mat frame;
        if (cap_.read(frame)) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_data_.image = frame.clone();
            latest_data_.timestamp = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }
    }
}

}
}