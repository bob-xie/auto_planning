#ifndef YOLO_DETECTOR_H
#define YOLO_DETECTOR_H

#include "PerceptionBase.h"
#include "PerceptionFactory.h"
#include "common/Config.h"
#include <opencv2/opencv.hpp>

namespace AD {
namespace Perception {

struct YOLOParams {
    std::string model_path;
    std::string config_path;
    std::string classes_path;
    float confidence_threshold = 0.5;
    float nms_threshold = 0.45;
    int input_width = 640;
    int input_height = 640;
};

class YOLODetector : public PerceptionBase {
public:
    explicit YOLODetector(const std::string& config_path);
    
    bool init(const std::string& config_path) override;
    DetectedObjects detect(const CameraData& camera_data, 
                           const LiDARData& lidar_data) override;
    std::string getName() const override { return "YOLO"; }

private:
    void loadClasses(const std::string& path);
    cv::Mat preprocessImage(const cv::Mat& image);
    DetectedObject::Type getObjectType(const std::string& class_name);
    
    YOLOParams params_;
    std::vector<std::string> classes_;
    cv::dnn::Net net_;
};

REGISTER_PERCEPTION("YOLO", YOLODetector);

}
}

#endif