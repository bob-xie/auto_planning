#ifndef PATH_H
#define PATH_H

#include <vector>
#include <Eigen/Dense>

namespace AD {

struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    double curvature = 0.0;
    double s = 0.0;
    
    Eigen::Vector3d getPosition() const {
        return Eigen::Vector3d(x, y, z);
    }
};

class Path {
public:
    std::vector<PathPoint> points;
    
    bool empty() const { return points.empty(); }
    
    size_t size() const { return points.size(); }
    
    const PathPoint& operator[](size_t idx) const {
        return points[idx];
    }
    
    PathPoint& operator[](size_t idx) {
        return points[idx];
    }
    
    void addPoint(double x, double y, double z = 0.0, 
                  double yaw = 0.0, double vel = 0.0) {
        PathPoint p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.yaw = yaw;
        p.velocity = vel;
        points.push_back(p);
    }
};

}

#endif