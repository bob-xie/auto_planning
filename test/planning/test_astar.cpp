#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <string>
#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include "planning/AStarPlanner.h"
#include "planning/smooth/SmootherFactory.h"
#include "common/VehicleState.h"
#include "common/Path.h"

static cv::Mat g_original_image;
static cv::Mat g_display_image;
static cv::Rect g_selection_rect;
static bool g_is_dragging = false;
static bool g_is_zoomed = false;
static cv::Point g_drag_start;

void resetView() {
    g_display_image = g_original_image.clone();
    g_is_zoomed = false;
    g_selection_rect = cv::Rect();
}

void zoomToRegion(const cv::Rect& region) {
    if (region.width < 10 || region.height < 10) return;
    
    cv::Mat cropped = g_original_image(region);
    cv::resize(cropped, g_display_image, g_original_image.size());
    g_is_zoomed = true;
}

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        g_drag_start = cv::Point(x, y);
        g_is_dragging = true;
        g_selection_rect = cv::Rect();
    } else if (event == cv::EVENT_MOUSEMOVE && g_is_dragging) {
        int w = x - g_drag_start.x;
        int h = y - g_drag_start.y;
        g_selection_rect = cv::Rect(g_drag_start.x, g_drag_start.y, w, h);
    } else if (event == cv::EVENT_LBUTTONUP) {
        g_is_dragging = false;
        if (g_selection_rect.width > 10 && g_selection_rect.height > 10) {
            zoomToRegion(g_selection_rect);
            std::cout << "\r已放大区域: (" << g_selection_rect.x << "," << g_selection_rect.y << ") - (" 
                      << g_selection_rect.x + g_selection_rect.width << "," 
                      << g_selection_rect.y + g_selection_rect.height << ")";
            std::cout.flush();
        }
        g_selection_rect = cv::Rect();
    }
}

void visualizePathComparison(const AD::Path& original_path,
                            const AD::Path& smoothed_path,
                            const AD::VehicleState& start, 
                            const AD::VehicleState& goal,
                            const std::vector<AD::Planning::Obstacle>& obstacles,
                            int image_size = 600) {
    double min_x = std::min(start.x, goal.x);
    double max_x = std::max(start.x, goal.x);
    double min_y = std::min(start.y, goal.y);
    double max_y = std::max(start.y, goal.y);
    
    for (const auto& obs : obstacles) {
        min_x = std::min(min_x, obs.x - obs.radius);
        max_x = std::max(max_x, obs.x + obs.radius);
        min_y = std::min(min_y, obs.y - obs.radius);
        max_y = std::max(max_y, obs.y + obs.radius);
    }
    
    for (const auto& point : original_path.points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    for (const auto& point : smoothed_path.points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    double padding = 15.0;
    min_x -= padding;
    max_x += padding;
    min_y -= padding;
    max_y += padding;
    
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    double scale = std::min(image_size / range_x, image_size / range_y);
    
    g_original_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    
    cv::rectangle(g_original_image, cv::Point(0, 0), cv::Point(image_size, image_size), 
                  cv::Scalar(240, 240, 240), -1);
    
    int grid_step = static_cast<int>(5.0 * scale);
    for (int x = 0; x < image_size; x += grid_step) {
        cv::line(g_original_image, cv::Point(x, 0), cv::Point(x, image_size), 
                 cv::Scalar(220, 220, 220), 1);
    }
    for (int y = 0; y < image_size; y += grid_step) {
        cv::line(g_original_image, cv::Point(0, y), cv::Point(image_size, y), 
                 cv::Scalar(220, 220, 220), 1);
    }
    
    for (size_t i = 0; i < obstacles.size(); i++) {
        int cx = static_cast<int>((obstacles[i].x - min_x) * scale);
        int cy = image_size - static_cast<int>((obstacles[i].y - min_y) * scale);
        int radius = static_cast<int>(obstacles[i].radius * scale);
        
        cv::circle(g_original_image, cv::Point(cx, cy), radius, cv::Scalar(0, 0, 255), -1);
        cv::circle(g_original_image, cv::Point(cx, cy), radius, cv::Scalar(0, 0, 180), 2);
    }
    
    if (!original_path.points.empty()) {
        for (size_t i = 1; i < original_path.points.size(); i++) {
            int x1 = static_cast<int>((original_path.points[i-1].x - min_x) * scale);
            int y1 = image_size - static_cast<int>((original_path.points[i-1].y - min_y) * scale);
            int x2 = static_cast<int>((original_path.points[i].x - min_x) * scale);
            int y2 = image_size - static_cast<int>((original_path.points[i].y - min_y) * scale);
            
            cv::line(g_original_image, cv::Point(x1, y1), cv::Point(x2, y2), 
                     cv::Scalar(255, 165, 0), 2, cv::LINE_AA);
        }
    }
    
    if (!smoothed_path.points.empty()) {
        for (size_t i = 1; i < smoothed_path.points.size(); i++) {
            int x1 = static_cast<int>((smoothed_path.points[i-1].x - min_x) * scale);
            int y1 = image_size - static_cast<int>((smoothed_path.points[i-1].y - min_y) * scale);
            int x2 = static_cast<int>((smoothed_path.points[i].x - min_x) * scale);
            int y2 = image_size - static_cast<int>((smoothed_path.points[i].y - min_y) * scale);
            
            cv::line(g_original_image, cv::Point(x1, y1), cv::Point(x2, y2), 
                     cv::Scalar(0, 200, 255), 3, cv::LINE_AA);
        }
    }
    
    int start_x = static_cast<int>((start.x - min_x) * scale);
    int start_y = image_size - static_cast<int>((start.y - min_y) * scale);
    cv::circle(g_original_image, cv::Point(start_x, start_y), 8, cv::Scalar(255, 0, 0), -1);
    cv::circle(g_original_image, cv::Point(start_x, start_y), 12, cv::Scalar(255, 0, 0), 2);
    
    int goal_x = static_cast<int>((goal.x - min_x) * scale);
    int goal_y = image_size - static_cast<int>((goal.y - min_y) * scale);
    cv::circle(g_original_image, cv::Point(goal_x, goal_y), 8, cv::Scalar(0, 0, 255), -1);
    cv::circle(g_original_image, cv::Point(goal_x, goal_y), 12, cv::Scalar(0, 0, 255), 2);
    
    int text_offset_x = (goal_x < image_size - 50) ? 15 : -60;
    int text_offset_y = (goal_y > 25) ? -15 : 30;
    cv::putText(g_original_image, "Start", cv::Point(start_x + 15, start_y - 15), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    cv::putText(g_original_image, "Goal", cv::Point(goal_x + text_offset_x, goal_y + text_offset_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    
    cv::putText(g_original_image, "A* Path Planning", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 100, 255), 2);
    
    int legend_x = 10;
    int legend_y = image_size - 70;
    
    cv::rectangle(g_original_image, cv::Point(legend_x, legend_y), 
                  cv::Point(legend_x + 160, legend_y + 50), 
                  cv::Scalar(255, 255, 255), -1);
    cv::rectangle(g_original_image, cv::Point(legend_x, legend_y), 
                  cv::Point(legend_x + 160, legend_y + 50), 
                  cv::Scalar(180, 180, 180), 1);
    
    cv::line(g_original_image, cv::Point(legend_x + 10, legend_y + 18), 
             cv::Point(legend_x + 40, legend_y + 18), 
             cv::Scalar(255, 165, 0), 2);
    cv::putText(g_original_image, "Original", cv::Point(legend_x + 45, legend_y + 22), 
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1);
    
    cv::line(g_original_image, cv::Point(legend_x + 10, legend_y + 38), 
             cv::Point(legend_x + 40, legend_y + 38), 
             cv::Scalar(0, 200, 255), 3);
    cv::putText(g_original_image, "Smoothed", cv::Point(legend_x + 45, legend_y + 42), 
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1);
    
    char info_str[100];
    sprintf(info_str, "Range: [%.1f,%.1f] x [%.1f,%.1f]", min_x, max_x, min_y, max_y);
    cv::putText(g_original_image, info_str, cv::Point(10, legend_y - 10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
    
    std::string output_file = "astar_path_comparison.png";
    bool saved = cv::imwrite(output_file, g_original_image);
    
    if (saved) {
        std::cout << "\n图像已保存到: " << output_file << std::endl;
    } else {
        std::cout << "\n警告: 无法保存图像文件" << std::endl;
    }
    
    std::cout << "\n=== 可视化窗口 ===" << std::endl;
    std::cout << "路径对比显示:" << std::endl;
    std::cout << "  橙色: 原始路径 (Original)" << std::endl;
    std::cout << "  青色: 平滑后路径 (Smoothed)" << std::endl;
    std::cout << "鼠标操作:" << std::endl;
    std::cout << "  拖拽选择区域: 放大选中区域" << std::endl;
    std::cout << "键盘操作:" << std::endl;
    std::cout << "  R     : 重置视图(恢复原始图像)" << std::endl;
    std::cout << "  Q / ESC: 退出" << std::endl;
    
    g_display_image = g_original_image.clone();
    g_is_zoomed = false;
    
    cv::namedWindow("A* Path Planning", cv::WINDOW_NORMAL);
    cv::resizeWindow("A* Path Planning", image_size, image_size);
    cv::setMouseCallback("A* Path Planning", onMouse, nullptr);
    cv::imshow("A* Path Planning", g_display_image);
    
    while (true) {
        cv::Mat show_image = g_display_image.clone();
        
        if (g_is_dragging && !g_selection_rect.empty()) {
            cv::rectangle(show_image, g_selection_rect, cv::Scalar(0, 255, 255), 2);
        }
        
        if (g_is_zoomed) {
            cv::putText(show_image, "已放大 - 按 R 重置", cv::Point(10, image_size - 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        }
        
        cv::imshow("A* Path Planning", show_image);
        
        int key = cv::waitKey(10);
        
        if (key == 'r' || key == 'R') {
            resetView();
            std::cout << "\r已重置视图";
            std::cout.flush();
        } else if (key == 'q' || key == 'Q' || key == 27) {
            std::cout << "\n退出可视化窗口" << std::endl;
            break;
        }
    }
    
    cv::destroyAllWindows();
}

void visualizePath(const AD::Path& path, 
                   const AD::VehicleState& start, 
                   const AD::VehicleState& goal,
                   const std::vector<AD::Planning::Obstacle>& obstacles,
                   int image_size = 600) {
    visualizePathComparison(path, path, start, goal, obstacles, image_size);
}

int main(int argc, char** argv) {
    std::cout << "=== 测试 A* 算法 ===" << std::endl;
    
    AD::Planning::AStarPlanner planner;
    planner.init(0.5, 0.5);
    
    double vehicle_length = 4.0;
    double vehicle_width = 1.8;
    double safety_distance = 0.5;
    
    planner.setVehicleDimensions(vehicle_length, vehicle_width);
    planner.setSafetyDistance(safety_distance);
    
    std::cout << "车辆尺寸: " << vehicle_length << "m x " << vehicle_width << "m" << std::endl;
    std::cout << "安全距离: " << safety_distance << "m" << std::endl;
    
    AD::VehicleState start;
    start.x = 0.0;
    start.y = 0.0;
    start.yaw = 0.0;
    
    AD::VehicleState goal;
    goal.x = 50.0;
    goal.y = 50.0;
    goal.yaw = 0.0;
    
    std::vector<AD::Planning::Obstacle> obstacles;
    obstacles.emplace_back(10.0, 10.0, 2.0);
    obstacles.emplace_back(20.0, 15.0, 1.5);
    obstacles.emplace_back(30.0, 5.0, 2.0);
    
    planner.setObstacles(obstacles);
    
    auto plan_start_time = std::chrono::high_resolution_clock::now();
    AD::Path path = planner.plan(start, goal);
    auto plan_end_time = std::chrono::high_resolution_clock::now();
    
    double plan_duration = std::chrono::duration<double, std::milli>(plan_end_time - plan_start_time).count();
    
    std::cout << "算法名称: " << planner.getName() << std::endl;
    std::cout << "路径长度: " << path.points.size() << " 个点" << std::endl;
    std::cout << "规划时间: " << plan_duration << " ms" << std::endl;
    
    if (!path.points.empty()) {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.points.size(); i++) {
            double dx = path.points[i].x - path.points[i-1].x;
            double dy = path.points[i].y - path.points[i-1].y;
            total_distance += std::sqrt(dx*dx + dy*dy);
        }
        std::cout << "路径总距离: " << total_distance << " m" << std::endl;
        std::cout << "起点: (" << path.points[0].x << ", " << path.points[0].y << ")" << std::endl;
        std::cout << "终点: (" << path.points.back().x << ", " << path.points.back().y << ")" << std::endl;
        
        double min_distance_to_obstacle = std::numeric_limits<double>::max();
        int closest_obs_idx = -1;
        int closest_point_idx = -1;
        
        for (size_t i = 0; i < path.points.size(); i++) {
            for (size_t j = 0; j < obstacles.size(); j++) {
                double dx = path.points[i].x - obstacles[j].x;
                double dy = path.points[i].y - obstacles[j].y;
                double dist = std::sqrt(dx*dx + dy*dy);
                
                if (dist < min_distance_to_obstacle) {
                    min_distance_to_obstacle = dist;
                    closest_obs_idx = j;
                    closest_point_idx = i;
                }
            }
        }
        
        std::cout << "\n障碍物距离分析:" << std::endl;
        std::cout << "最小距离到障碍物: " << std::fixed << std::setprecision(2) 
                  << min_distance_to_obstacle << " m" << std::endl;
        std::cout << "最近障碍物索引: " << closest_obs_idx 
                  << " (位置: (" << obstacles[closest_obs_idx].x << ", " 
                  << obstacles[closest_obs_idx].y << "), 半径: " 
                  << obstacles[closest_obs_idx].radius << "m)" << std::endl;
        std::cout << "路径上最近点: (" << path.points[closest_point_idx].x << ", " 
                  << path.points[closest_point_idx].y << ")" << std::endl;
        
        double required_clearance = std::max(vehicle_length, vehicle_width) / 2.0 + safety_distance;
        std::cout << "所需安全距离: " << std::fixed << std::setprecision(2) << required_clearance << " m" << std::endl;
        
        if (min_distance_to_obstacle >= obstacles[closest_obs_idx].radius + required_clearance) {
            std::cout << "✓ 路径安全，满足安全距离要求" << std::endl;
        } else {
            std::cout << "✗ 路径存在碰撞风险，建议增大安全距离或调整障碍物位置" << std::endl;
        }
    }
    
    std::cout << "\n=== 路径平滑处理 ===" << std::endl;
    std::cout << "选择平滑算法:" << std::endl;
    std::cout << "  1. B-Spline Smoother (B样条平滑)" << std::endl;
    std::cout << "  2. Window Smoother (窗口加权平滑)" << std::endl;
    std::cout << "  3. Gradient Descent Smoother (梯度下降平滑)" << std::endl;
    std::cout << "输入选择 (1/2/3): ";
    
    int choice;
    std::cin >> choice;
    
    AD::Planning::Smooth::SmootherType smoother_type;
    switch (choice) {
        case 2:
            smoother_type = AD::Planning::Smooth::SmootherType::WINDOW;
            break;
        case 3:
            smoother_type = AD::Planning::Smooth::SmootherType::GRADIENT_DESCENT;
            break;
        default:
            smoother_type = AD::Planning::Smooth::SmootherType::BSPLINE;
    }
    
    auto smoother = AD::Planning::Smooth::SmootherFactory::create(smoother_type);
    smoother->init(0.5, 0.5);
    
    auto smooth_start_time = std::chrono::high_resolution_clock::now();
    AD::Path smoothed_path = smoother->smooth(path);
    auto smooth_end_time = std::chrono::high_resolution_clock::now();
    
    double smooth_duration = std::chrono::duration<double, std::milli>(smooth_end_time - smooth_start_time).count();
    
    std::cout << "平滑算法: " << smoother->getName() << std::endl;
    std::cout << "平滑前路径点数: " << path.points.size() << std::endl;
    std::cout << "平滑后路径点数: " << smoothed_path.points.size() << std::endl;
    std::cout << "平滑时间: " << smooth_duration << " ms" << std::endl;
    
    if (!smoothed_path.points.empty()) {
        double smooth_total_distance = 0.0;
        for (size_t i = 1; i < smoothed_path.points.size(); i++) {
            double dx = smoothed_path.points[i].x - smoothed_path.points[i-1].x;
            double dy = smoothed_path.points[i].y - smoothed_path.points[i-1].y;
            smooth_total_distance += std::sqrt(dx*dx + dy*dy);
        }
        std::cout << "平滑后路径总距离: " << smooth_total_distance << " m" << std::endl;
    }
    
    visualizePathComparison(path, smoothed_path, start, goal, obstacles);
    
    std::cout << "A* 测试完成" << std::endl;
    return 0;
}