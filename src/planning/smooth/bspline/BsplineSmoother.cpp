#include "planning/smooth/bspline/BsplineSmoother.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace AD {
namespace Planning {
namespace Smooth {

bool BsplineSmoother::init(double smoothing_factor, double resolution) {
    smoothing_factor_ = smoothing_factor;
    resolution_ = resolution;
    return true;
}

void BsplineSmoother::computeCumulativeLengths(const std::vector<Eigen::Vector2d>& points, 
                                               std::vector<double>& lengths) {
    lengths.resize(points.size());
    lengths[0] = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        lengths[i] = lengths[i-1] + (points[i] - points[i-1]).norm();
    }
}

Eigen::Vector2d BsplineSmoother::lerp(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double t) {
    return a + (b - a) * t;
}

Path BsplineSmoother::smooth(const Path& input_path) {
    Path smoothed_path;
    
    if (input_path.points.empty()) {
        return smoothed_path;
    }
    
    if (input_path.points.size() < 3) {
        return input_path;
    }
    
    std::vector<Eigen::Vector2d> points;
    for (const auto& pt : input_path.points) {
        points.emplace_back(pt.x, pt.y);
    }
    
    int n = points.size();
    std::vector<Eigen::Vector2d> smoothed(n);
    
    for (int i = 0; i < n; ++i) {
        smoothed[i] = points[i];
    }
    
    int iterations = static_cast<int>(10.0 * smoothing_factor_);
    double weight_data = 0.5;
    double weight_smooth = 0.3;
    
    for (int iter = 0; iter < iterations; ++iter) {
        for (int i = 1; i < n - 1; ++i) {
            Eigen::Vector2d prev = smoothed[i-1];
            Eigen::Vector2d curr = smoothed[i];
            Eigen::Vector2d next = smoothed[i+1];
            
            Eigen::Vector2d diff = prev + next - 2.0 * curr;
            Eigen::Vector2d data_term = points[i] - curr;
            
            smoothed[i] += weight_data * data_term + weight_smooth * diff;
        }
    }
    
    std::vector<double> lengths;
    computeCumulativeLengths(smoothed, lengths);
    double total_length = lengths.back();
    
    int num_samples = std::max(static_cast<int>(total_length / resolution_) + 1, 50);
    
    for (int i = 0; i < num_samples; ++i) {
        double target_len = (total_length * i) / (num_samples - 1);
        
        int idx = 0;
        for (int j = 0; j < n - 1; ++j) {
            if (lengths[j] <= target_len && target_len <= lengths[j+1]) {
                idx = j;
                break;
            }
        }
        
        double t = (target_len - lengths[idx]) / (lengths[idx+1] - lengths[idx]);
        Eigen::Vector2d pt = lerp(smoothed[idx], smoothed[idx+1], t);
        
        smoothed_path.addPoint(pt(0), pt(1), 0.0, 0.0, 10.0);
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