#include "planning/smooth/window/WindowSmoother.h"
#include <cmath>

namespace AD {
namespace Planning {
namespace Smooth {

bool WindowSmoother::init(double window_size, double weight) {
    window_size_ = static_cast<int>(window_size);
    weight_ = weight;
    return true;
}

Path WindowSmoother::smooth(const Path& input_path) {
    Path smoothed_path;
    
    if (input_path.points.empty()) {
        return smoothed_path;
    }
    
    if (input_path.points.size() <= window_size_) {
        return input_path;
    }
    
    smoothed_path.points.resize(input_path.points.size());
    
    for (size_t i = 0; i < input_path.points.size(); ++i) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_weights = 0.0;
        
        int start_idx = std::max(0, static_cast<int>(i) - window_size_ / 2);
        int end_idx = std::min(static_cast<int>(input_path.points.size()) - 1, 
                               static_cast<int>(i) + window_size_ / 2);
        
        for (int j = start_idx; j <= end_idx; ++j) {
            double dist = std::abs(j - static_cast<int>(i));
            double w = std::exp(-dist * dist / (2.0 * weight_ * weight_));
            
            sum_x += input_path.points[j].x * w;
            sum_y += input_path.points[j].y * w;
            sum_weights += w;
        }
        
        if (sum_weights > 0) {
            smoothed_path.points[i].x = sum_x / sum_weights;
            smoothed_path.points[i].y = sum_y / sum_weights;
        } else {
            smoothed_path.points[i].x = input_path.points[i].x;
            smoothed_path.points[i].y = input_path.points[i].y;
        }
        
        smoothed_path.points[i].velocity = input_path.points[i].velocity;
        smoothed_path.points[i].curvature = input_path.points[i].curvature;
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