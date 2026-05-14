#ifndef POINTPILLARS_DETECTOR_H
#define POINTPILLARS_DETECTOR_H

#include "PerceptionBase.h"
#include "PerceptionFactory.h"
#include "common/Config.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace AD {
namespace Perception {

struct PointPillarsParams {
    std::string model_path;
    float confidence_threshold = 0.5;
    float nms_threshold = 0.5;
    
    float voxel_size_x = 0.16;
    float voxel_size_y = 0.16;
    float voxel_size_z = 0.4;
    
    float point_cloud_range_min_x = -51.2;
    float point_cloud_range_max_x = 51.2;
    float point_cloud_range_min_y = -51.2;
    float point_cloud_range_max_y = 51.2;
    float point_cloud_range_min_z = -3.0;
    float point_cloud_range_max_z = 1.0;
    
    int max_points_per_voxel = 32;
    int max_voxels = 40000;
};

class PointPillarsDetector : public PerceptionBase {
public:
    explicit PointPillarsDetector(const std::string& config_path);
    
    bool init(const std::string& config_path) override;
    DetectedObjects detect(const CameraData& camera_data, 
                           const LiDARData& lidar_data) override;
    std::string getName() const override { return "PointPillars"; }

private:
    std::vector<float> preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    DetectedObject::Type getObjectType(int class_id);
    
    PointPillarsParams params_;
    std::vector<std::string> classes_ = {"Car", "Pedestrian", "Cyclist"};
};

REGISTER_PERCEPTION("PointPillars", PointPillarsDetector);

}
}

#endif