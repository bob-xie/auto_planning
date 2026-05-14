#ifndef RTK_GPS_H
#define RTK_GPS_H

#include <string>

namespace AD {
namespace Localization {

struct GPSData {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double vel_n = 0.0;
    double vel_e = 0.0;
    double vel_d = 0.0;
    double heading = 0.0;
    int fix_type = 0;
    double timestamp = 0.0;
};

class RTKGPS {
public:
    bool init(const std::string& port = "/dev/ttyUSB0", int baud = 115200);
    void start();
    void stop();
    
    GPSData getLatestData();

private:
    void parseNMEA(const std::string& nmea);
    
    GPSData latest_data_;
    bool running_ = false;
};

}
}

#endif
