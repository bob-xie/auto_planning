#include "common/Config.h"

namespace AD {

bool Config::load(const std::string& filename) {
    try {
        config_ = YAML::LoadFile(filename);
        return true;
    } catch (const YAML::Exception& e) {
        return false;
    }
}

}