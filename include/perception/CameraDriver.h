#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <opencv2/opencv.hpp>
#include <mutex>

namespace AD {
namespace Perception {

struct CameraData {
    cv::Mat image;
    double timestamp = 0.0;
};

class CameraDriver {
public:
    bool init(const std::string& device = "/dev/video0");
    void start();
    void stop();
    
    CameraData getLatestData();

private:
    void captureLoop();
    
    cv::VideoCapture cap_;
    CameraData latest_data_;
    std::mutex mutex_;
    bool running_ = false;
};

}
}

#endif