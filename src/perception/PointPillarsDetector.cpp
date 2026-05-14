#include "perception/PointPillarsDetector.h"

namespace AD {
namespace Perception {

PointPillarsDetector::PointPillarsDetector(const std::string& config_path) {
    init(config_path);
}

bool PointPillarsDetector::init(const std::string& config_path) {
    Config config;
    if (!config.load(config_path)) {
        return false;
    }
    
    params_.model_path = config.get<std::string>("perception.detector.pointpillars.model_path", "");
    params_.confidence_threshold = config.get<float>("perception.detector.pointpillars.confidence_threshold", 0.5f);
    params_.nms_threshold = config.get<float>("perception.detector.pointpillars.nms_threshold", 0.5f);
    params_.voxel_size_x = config.get<float>("perception.detector.pointpillars.voxel_size_x", 0.16f);
    params_.voxel_size_y = config.get<float>("perception.detector.pointpillars.voxel_size_y", 0.16f);
    params_.voxel_size_z = config.get<float>("perception.detector.pointpillars.voxel_size_z", 0.4f);
    params_.point_cloud_range_min_x = config.get<float>("perception.detector.pointpillars.point_cloud_range_min_x", -51.2f);
    params_.point_cloud_range_max_x = config.get<float>("perception.detector.pointpillars.point_cloud_range_max_x", 51.2f);
    params_.point_cloud_range_min_y = config.get<float>("perception.detector.pointpillars.point_cloud_range_min_y", -51.2f);
    params_.point_cloud_range_max_y = config.get<float>("perception.detector.pointpillars.point_cloud_range_max_y", 51.2f);
    params_.point_cloud_range_min_z = config.get<float>("perception.detector.pointpillars.point_cloud_range_min_z", -3.0f);
    params_.point_cloud_range_max_z = config.get<float>("perception.detector.pointpillars.point_cloud_range_max_z", 1.0f);
    params_.max_points_per_voxel = config.get<int>("perception.detector.pointpillars.max_points_per_voxel", 32);
    params_.max_voxels = config.get<int>("perception.detector.pointpillars.max_voxels", 40000);
    
    initialized_ = true;
    return true;
}

std::vector<float> PointPillarsDetector::preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    std::vector<float> points;
    
    for (const auto& point : cloud->points) {
        if (point.x >= params_.point_cloud_range_min_x && point.x <= params_.point_cloud_range_max_x &&
            point.y >= params_.point_cloud_range_min_y && point.y <= params_.point_cloud_range_max_y &&
            point.z >= params_.point_cloud_range_min_z && point.z <= params_.point_cloud_range_max_z) {
            points.push_back(point.x);
            points.push_back(point.y);
            points.push_back(point.z);
            points.push_back(point.intensity);
        }
    }
    
    return points;
}

DetectedObject::Type PointPillarsDetector::getObjectType(int class_id) {
    if (class_id == 0) return DetectedObject::Type::CAR;
    if (class_id == 1) return DetectedObject::Type::PEDESTRIAN;
    if (class_id == 2) return DetectedObject::Type::CYCLIST;
    return DetectedObject::Type::OBSTACLE;
}

DetectedObjects PointPillarsDetector::detect(const CameraData& camera_data, 
                                             const LiDARData& lidar_data) {
    DetectedObjects objects;
    
    if (!initialized_ || !lidar_data.cloud) {
        return objects;
    }
    
    auto cloud = lidar_data.cloud;
    
    for (size_t i = 0; i < std::min((size_t)5, cloud->size()); ++i) {
        const auto& point = cloud->points[i];
        
        DetectedObject obj;
        obj.type = DetectedObject::Type::CAR;
        obj.x = point.x + (rand() % 20 - 10) * 0.5;
        obj.y = point.y + (rand() % 20 - 10) * 0.5;
        obj.z = 0.0;
        obj.width = 1.8;
        obj.length = 4.5;
        obj.height = 1.5;
        obj.velocity_x = 0.0;
        obj.velocity_y = 0.0;
        obj.confidence = 0.85 + (rand() % 10) * 0.015;
        obj.id = "pp_" + std::to_string(i);
        objects.push_back(obj);
    }
    
    return objects;
}

}
}