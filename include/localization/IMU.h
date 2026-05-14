#ifndef IMU_H
#define IMU_H

#include <Eigen/Dense>
#include <string>

namespace AD {
namespace Localization {

struct IMUData {
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d orientation;
    double timestamp = 0.0;
};

class IMU {
public:
    bool init(const std::string& port = "/dev/ttyUSB1");
    void start();
    void stop();
    
    IMUData getLatestData();

private:
    void readLoop();
    
    IMUData latest_data_;
    bool running_ = false;
};

}
}

#endif
