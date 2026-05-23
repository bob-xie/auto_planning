#include "localization/RTKGPS.h"
#include <string>

namespace AD {
namespace Localization {

bool RTKGPS::init(const std::string& port, int baud) {
    return true;
}

void RTKGPS::start() {
    running_ = true;
}

void RTKGPS::stop() {
    running_ = false;
}

GPSData RTKGPS::getLatestData() {
    return latest_data_;
}

void RTKGPS::parseNMEA(const std::string& nmea) {
}

}
}
