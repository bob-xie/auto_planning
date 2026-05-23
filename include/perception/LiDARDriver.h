#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

namespace AD {
namespace Perception {

struct LiDARData {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double timestamp = 0.0;
};

class LiDARDriver {
public:
    bool init(const std::string& ip = "192.168.1.201", int port = 2368);
    void start();
    void stop();
    
    LiDARData getLatestData();

private:
    void receiveLoop();
    
    LiDARData latest_data_;
    std::mutex mutex_;
    bool running_ = false;
};

}
}

#endif
