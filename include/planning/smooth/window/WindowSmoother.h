#ifndef WINDOW_SMOOTHER_H
#define WINDOW_SMOOTHER_H

#include "planning/smooth/PathSmoother.h"

namespace AD {
namespace Planning {
namespace Smooth {

class WindowSmoother : public PathSmoother {
public:
    bool init(double window_size = 5.0, double weight = 0.5) override;
    
    Path smooth(const Path& input_path) override;
    
    std::string getName() const override { return "Window Smoother"; }
    
private:
    int window_size_;
    double weight_;
};

}
}
}

#endif