#include "planning/smooth/gradient_descent/GradientDescentSmoother.h"
#include <cmath>
#include <algorithm>

namespace AD {
namespace Planning {
namespace Smooth {

bool GradientDescentSmoother::init(double learning_rate, double weight_smooth) {
    learning_rate_ = learning_rate;
    weight_smooth_ = weight_smooth;
    return true;
}

void GradientDescentSmoother::setParams(double weight_data, int max_iterations, double tolerance) {
    weight_data_ = weight_data;
    max_iterations_ = max_iterations;
    tolerance_ = tolerance;
}

Path GradientDescentSmoother::smooth(const Path& input_path) {
    Path smoothed_path;
    
    if (input_path.points.empty()) {
        return smoothed_path;
    }
    
    if (input_path.points.size() < 3) {
        return input_path;
    }
    
    int n = input_path.points.size();
    
    std::vector<double> x(n), y(n);
    std::vector<double> x_new(n), y_new(n);
    
    for (int i = 0; i < n; ++i) {
        x[i] = input_path.points[i].x;
        y[i] = input_path.points[i].y;
        x_new[i] = x[i];
        y_new[i] = y[i];
    }
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        double max_delta = 0.0;
        
        for (int i = 1; i < n - 1; ++i) {
            double grad_x = weight_data_ * (x[i] - input_path.points[i].x);
            grad_x += weight_smooth_ * (2.0 * x[i] - x[i-1] - x[i+1]);
            
            double grad_y = weight_data_ * (y[i] - input_path.points[i].y);
            grad_y += weight_smooth_ * (2.0 * y[i] - y[i-1] - y[i+1]);
            
            x_new[i] = x[i] - learning_rate_ * grad_x;
            y_new[i] = y[i] - learning_rate_ * grad_y;
            
            double delta = std::sqrt(std::pow(x_new[i] - x[i], 2) + std::pow(y_new[i] - y[i], 2));
            max_delta = std::max(max_delta, delta);
        }
        
        std::copy(x_new.begin(), x_new.end(), x.begin());
        std::copy(y_new.begin(), y_new.end(), y.begin());
        
        if (max_delta < tolerance_) {
            break;
        }
    }
    
    for (int i = 0; i < n; ++i) {
        smoothed_path.addPoint(x[i], y[i], 0.0, 0.0, 10.0);
    }
    
    for (size_t i = 0; i < smoothed_path.points.size(); ++i) {
        double yaw = 0.0;
        if (i < smoothed_path.points.size() - 1) {
            double dx = smoothed_path.points[i+1].x - smoothed_path.points[i].x;
            double dy = smoothed_path.points[i+1].y - smoothed_path.points[i].y;
            yaw = std::atan2(dy, dx);
        } else if (i > 0) {
            double dx = smoothed_path.points[i].x - smoothed_path.points[i-1].x;
            double dy = smoothed_path.points[i].y - smoothed_path.points[i-1].y;
            yaw = std::atan2(dy, dx);
        }
        smoothed_path.points[i].yaw = yaw;
    }
    
    return smoothed_path;
}

}
}
}