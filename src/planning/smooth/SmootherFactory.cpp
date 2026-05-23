#include "planning/smooth/SmootherFactory.h"
#include "planning/smooth/bspline/BsplineSmoother.h"
#include "planning/smooth/window/WindowSmoother.h"
#include "planning/smooth/gradient_descent/GradientDescentSmoother.h"
#include <algorithm>
#include <cstring>

namespace AD {
namespace Planning {
namespace Smooth {

std::unique_ptr<PathSmoother> SmootherFactory::create(SmootherType type) {
    switch (type) {
        case SmootherType::BSPLINE:
            return std::make_unique<BsplineSmoother>();
        case SmootherType::WINDOW:
            return std::make_unique<WindowSmoother>();
        case SmootherType::GRADIENT_DESCENT:
            return std::make_unique<GradientDescentSmoother>();
        default:
            return std::make_unique<BsplineSmoother>();
    }
}

std::unique_ptr<PathSmoother> SmootherFactory::create(const std::string& type_name) {
    return create(stringToType(type_name));
}

std::vector<std::string> SmootherFactory::getAvailableSmoothers() {
    return {
        "bspline",
        "window", 
        "gradient_descent"
    };
}

SmootherType SmootherFactory::stringToType(const std::string& type_name) {
    std::string lower_name = type_name;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
    
    if (lower_name == "bspline" || lower_name == "b-spline") {
        return SmootherType::BSPLINE;
    } else if (lower_name == "window") {
        return SmootherType::WINDOW;
    } else if (lower_name == "gradient_descent" || lower_name == "gradient") {
        return SmootherType::GRADIENT_DESCENT;
    }
    
    return SmootherType::BSPLINE;
}

std::string SmootherFactory::typeToString(SmootherType type) {
    switch (type) {
        case SmootherType::BSPLINE:
            return "bspline";
        case SmootherType::WINDOW:
            return "window";
        case SmootherType::GRADIENT_DESCENT:
            return "gradient_descent";
        default:
            return "bspline";
    }
}

}
}
}