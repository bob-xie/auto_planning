#ifndef SMOOTHER_FACTORY_H
#define SMOOTHER_FACTORY_H

#include "planning/smooth/PathSmoother.h"
#include <memory>
#include <string>
#include <vector>

namespace AD {
namespace Planning {
namespace Smooth {

enum class SmootherType {
    BSPLINE,
    WINDOW,
    GRADIENT_DESCENT
};

class SmootherFactory {
public:
    static std::unique_ptr<PathSmoother> create(SmootherType type);
    
    static std::unique_ptr<PathSmoother> create(const std::string& type_name);
    
    static std::vector<std::string> getAvailableSmoothers();
    
    static SmootherType stringToType(const std::string& type_name);
    
    static std::string typeToString(SmootherType type);
};

}
}
}

#endif