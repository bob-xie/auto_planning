#ifndef BSPLINE_SMOOTHER_H
#define BSPLINE_SMOOTHER_H

#include "planning/smooth/PathSmoother.h"
#include "common/Path.h"
#include <Eigen/Dense>
#include <vector>

namespace AD {
namespace Planning {
namespace Smooth {

class BsplineSmoother : public PathSmoother {
public:
    BsplineSmoother() : smoothing_factor_(0.6), resolution_(0.2) {}
    
    bool init(double smoothing_factor = 0.6, double resolution = 0.2) override;
    
    Path smooth(const Path& input_path) override;
    
    std::string getName() const override { return "B-Spline Smoother"; }

private:
    double smoothing_factor_;
    double resolution_;
    
    void computeCumulativeLengths(const std::vector<Eigen::Vector2d>& points, 
                                  std::vector<double>& lengths);
    
    Eigen::Vector2d lerp(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double t);
};

}
}
}

#endif