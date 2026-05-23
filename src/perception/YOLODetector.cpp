#include "perception/YOLODetector.h"
#include <fstream>
#include <sstream>

namespace AD {
namespace Perception {

YOLODetector::YOLODetector(const std::string& config_path) {
    init(config_path);
}

bool YOLODetector::init(const std::string& config_path) {
    Config config;
    if (!config.load(config_path)) {
        return false;
    }
    
    params_.model_path = config.get<std::string>("perception.detector.yolo.model_path", "");
    params_.config_path = config.get<std::string>("perception.detector.yolo.config_path", "");
    params_.classes_path = config.get<std::string>("perception.detector.yolo.classes_path", "");
    params_.confidence_threshold = config.get<float>("perception.detector.yolo.confidence_threshold", 0.5f);
    params_.nms_threshold = config.get<float>("perception.detector.yolo.nms_threshold", 0.45f);
    params_.input_width = config.get<int>("perception.detector.yolo.input_width", 640);
    params_.input_height = config.get<int>("perception.detector.yolo.input_height", 640);
    
    loadClasses(params_.classes_path);
    
    if (!params_.model_path.empty() && !params_.config_path.empty()) {
        net_ = cv::dnn::readNetFromDarknet(params_.config_path, params_.model_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    
    initialized_ = true;
    return true;
}

void YOLODetector::loadClasses(const std::string& path) {
    if (path.empty()) {
        classes_ = {"person", "bicycle", "car", "motorcycle", "airplane", 
                    "bus", "train", "truck", "boat", "traffic light"};
        return;
    }
    
    std::ifstream file(path);
    if (!file.is_open()) {
        return;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        classes_.push_back(line);
    }
}

cv::Mat YOLODetector::preprocessImage(const cv::Mat& image) {
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1/255.0, 
                           cv::Size(params_.input_width, params_.input_height), 
                           cv::Scalar(0, 0, 0), true, false);
    return blob;
}

DetectedObject::Type YOLODetector::getObjectType(const std::string& class_name) {
    if (class_name == "person") return DetectedObject::Type::PEDESTRIAN;
    if (class_name == "bicycle" || class_name == "motorcycle") return DetectedObject::Type::CYCLIST;
    if (class_name == "car" || class_name == "bus" || class_name == "truck") return DetectedObject::Type::CAR;
    return DetectedObject::Type::OBSTACLE;
}

DetectedObjects YOLODetector::detect(const CameraData& camera_data, 
                                     const LiDARData& lidar_data) {
    DetectedObjects objects;
    
    if (!initialized_ || camera_data.image.empty()) {
        return objects;
    }
    
    cv::Mat blob = preprocessImage(camera_data.image);
    net_.setInput(blob);
    
    std::vector<cv::Mat> outs;
    std::vector<std::string> outNames = net_.getUnconnectedOutLayersNames();
    net_.forward(outs, outNames);
    
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    for (const auto& out : outs) {
        float* data = (float*)out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols) {
            cv::Mat scores = out.row(j).colRange(5, out.cols);
            cv::Point classIdPoint;
            double confidence;
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            
            if (confidence > params_.confidence_threshold) {
                int centerX = (int)(data[0] * camera_data.image.cols);
                int centerY = (int)(data[1] * camera_data.image.rows);
                int width = (int)(data[2] * camera_data.image.cols);
                int height = (int)(data[3] * camera_data.image.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, params_.confidence_threshold, params_.nms_threshold, indices);
    
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        DetectedObject obj;
        obj.type = getObjectType(classes_[classIds[idx]]);
        obj.x = boxes[idx].x + boxes[idx].width / 2.0;
        obj.y = boxes[idx].y + boxes[idx].height / 2.0;
        obj.z = 0.0;
        obj.width = boxes[idx].width;
        obj.length = boxes[idx].height;
        obj.height = 1.5;
        obj.velocity_x = 0.0;
        obj.velocity_y = 0.0;
        obj.confidence = confidences[idx];
        obj.id = "yolo_" + std::to_string(i);
        objects.push_back(obj);
    }
    
    return objects;
}

}
}