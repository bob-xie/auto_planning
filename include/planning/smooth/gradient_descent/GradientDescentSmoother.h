#ifndef GRADIENT_DESCENT_SMOOTHER_H
#define GRADIENT_DESCENT_SMOOTHER_H

#include "planning/smooth/PathSmoother.h"
#include "common/Path.h"
#include <Eigen/Dense>
#include <vector>

namespace AD {
namespace Planning {
namespace Smooth {

class GradientDescentSmoother : public PathSmoother {
public:
    GradientDescentSmoother() 
        : learning_rate_(0.1), weight_smooth_(0.5), weight_data_(0.5), 
          max_iterations_(100), tolerance_(1e-6) {}
    
    bool init(double learning_rate = 0.1, double weight_smooth = 0.5) override;
    
    void setParams(double weight_data, int max_iterations, double tolerance);
    
    Path smooth(const Path& input_path) override;
    
    std::string getName() const override { return "Gradient Descent Smoother"; }

private:
    double learning_rate_;
    double weight_smooth_;
    double weight_data_;
    int max_iterations_;
    double tolerance_;
};

}
}
}

#endif