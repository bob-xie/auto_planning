#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <string>
#include <limits>
#include <memory>
#include <thread>
#include <atomic>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "planning/RRTPlanner.h"
#include "planning/smooth/SmootherFactory.h"
#include "common/VehicleState.h"
#include "common/Path.h"
#include "fox_visual/FoxgloveVisualizer.h"

static cv::Mat g_original_image;
static cv::Mat g_display_image;
static cv::Rect g_selection_rect;
static bool g_is_dragging = false;
static bool g_is_zoomed = false;
static cv::Point g_drag_start;
static std::atomic<bool> g_simulation_running(true);

class McapWriter {
public:
    McapWriter() : process_(nullptr), is_open_(false) {}

    ~McapWriter() { close(); }

    bool open(const std::string& mcap_path) {
        close();

        std::string helper_script = "../mcap_writer_helper.py";
        std::string command = "python3 " + helper_script + " " + mcap_path + " 2>&1";

        process_ = popen(command.c_str(), "w");
        if (!process_) {
            std::cerr << "Failed to start MCAP writer process" << std::endl;
            return false;
        }

        is_open_ = true;
        return true;
    }

    void close() {
        if (process_) {
            pclose(process_);
            process_ = nullptr;
        }
        is_open_ = false;
    }

    bool isOpen() const { return is_open_; }

    void writeMessage(const std::string& topic, const std::string& json_data) {
        if (!process_) return;

        std::stringstream ss;
        ss << "{\"topic\": \"" << topic << "\", \"data\": " << json_data << "}\n";

        fputs(ss.str().c_str(), process_);
        fflush(process_);
    }

private:
    FILE* process_;
    bool is_open_;
};

class PathTracker {
public:
    PathTracker(double max_speed, double max_acceleration)
        : max_speed_(max_speed), max_acceleration_(max_acceleration),
          current_speed_(0.0), target_speed_(max_speed) {}

    double getMaxSpeed() const { return max_speed_; }
    double getCurrentSpeed() const { return current_speed_; }

    void updateSpeed(double dt) {
        if (current_speed_ < target_speed_) {
            current_speed_ += max_acceleration_ * dt;
            if (current_speed_ > target_speed_) {
                current_speed_ = target_speed_;
            }
        } else if (current_speed_ > target_speed_) {
            current_speed_ -= max_acceleration_ * dt;
            if (current_speed_ < target_speed_) {
                current_speed_ = target_speed_;
            }
        }
    }

    void setTargetSpeed(double speed) {
        target_speed_ = std::max(0.0, std::min(speed, max_speed_));
    }

private:
    double max_speed_;
    double max_acceleration_;
    double current_speed_;
    double target_speed_;
};

struct Point2D {
    double x, y, z;
    Point2D(double x_=0, double y_=0, double z_=0) : x(x_), y(y_), z(z_) {}
};

class VehicleSimulator {
public:
    VehicleSimulator()
        : position_(0, 0, 0), yaw_(0),
          tracker_(5.0, 1.0),
          current_path_index_(0),
          lookahead_distance_(3.0),
          distance_traveled_(0.0),
          replan_distance_(10.0) {}

    void init(const Point2D& start_pos, double start_yaw) {
        position_ = start_pos;
        yaw_ = start_yaw;
        current_path_index_ = 0;
        distance_traveled_ = 0.0;
    }

    void setPath(const std::vector<Point2D>& path, bool is_original) {
        if (is_original) {
            original_path_ = path;
        } else {
            smoothed_path_ = path;
        }
        current_path_index_ = findClosestPointIndex(position_);
        distance_traveled_ = 0.0;
    }

    void setReplanDistance(double dist) {
        replan_distance_ = dist;
    }

    bool update(double dt) {
        const auto& path = smoothed_path_;

        if (path.empty()) {
            return false;
        }

        tracker_.updateSpeed(dt);

        current_path_index_ = findClosestPointIndex(position_);
        Point2D lookahead_point = findLookaheadPoint(path);

        double dx = lookahead_point.x - position_.x;
        double dy = lookahead_point.y - position_.y;
        double target_yaw = std::atan2(dy, dx);

        double yaw_diff = target_yaw - yaw_;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

        double max_yaw_rate = M_PI / 3;
        if (std::abs(yaw_diff) > max_yaw_rate * dt) {
            yaw_ += (yaw_diff > 0 ? max_yaw_rate : -max_yaw_rate) * dt;
        } else {
            yaw_ += yaw_diff;
        }

        while (yaw_ > M_PI) yaw_ -= 2 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2 * M_PI;

        double speed = tracker_.getCurrentSpeed();
        double move_distance = speed * dt;

        Point2D last_pos = position_;
        position_.x += std::cos(yaw_) * move_distance;
        position_.y += std::sin(yaw_) * move_distance;

        double dist_moved = std::sqrt(
            std::pow(position_.x - last_pos.x, 2) +
            std::pow(position_.y - last_pos.y, 2));
        distance_traveled_ += dist_moved;

        double dist_to_goal = std::sqrt(
            std::pow(path.back().x - position_.x, 2) +
            std::pow(path.back().y - position_.y, 2));

        if (dist_to_goal < 2.0) {
            return false;
        }

        return true;
    }

    bool needsReplan() const {
        return distance_traveled_ >= replan_distance_;
    }

    void resetReplanDistance() {
        distance_traveled_ = 0.0;
    }

    Point2D getPosition() const { return position_; }
    double getYaw() const { return yaw_; }
    double getCurrentSpeed() const { return tracker_.getCurrentSpeed(); }
    double getDistanceTraveled() const { return distance_traveled_; }
    size_t getPathIndex() const { return current_path_index_; }

    const auto& getOriginalPath() const { return original_path_; }
    const auto& getSmoothedPath() const { return smoothed_path_; }

private:
    size_t findClosestPointIndex(const Point2D& pos) const {
        const auto& path = smoothed_path_;
        if (path.empty()) return 0;

        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = current_path_index_;

        size_t start_idx = current_path_index_ > 0 ? current_path_index_ - 1 : 0;
        size_t end_idx = std::min(current_path_index_ + 10, path.size());

        for (size_t i = start_idx; i < end_idx; ++i) {
            double dx = path[i].x - pos.x;
            double dy = path[i].y - pos.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    Point2D findLookaheadPoint(const std::vector<Point2D>& path) const {
        if (path.empty()) return position_;

        double accumulated_dist = 0.0;

        for (size_t i = current_path_index_; i < path.size() - 1; ++i) {
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            double segment_dist = std::sqrt(dx * dx + dy * dy);

            if (accumulated_dist + segment_dist >= lookahead_distance_) {
                double t = (lookahead_distance_ - accumulated_dist) / segment_dist;
                return Point2D(
                    path[i].x + dx * t,
                    path[i].y + dy * t,
                    path[i].z);
            }

            accumulated_dist += segment_dist;
        }

        return path.back();
    }

    Point2D position_;
    double yaw_;
    PathTracker tracker_;
    std::vector<Point2D> original_path_;
    std::vector<Point2D> smoothed_path_;
    size_t current_path_index_;
    double lookahead_distance_;
    double distance_traveled_;
    double replan_distance_;
};

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

void visualizeSimulation(const std::vector<AD::Planning::Obstacle>& obstacles,
                       const VehicleSimulator& vehicle,
                       const AD::VehicleState& initial_start,
                       const AD::VehicleState& goal,
                       double scale,
                       double min_x,
                       double min_y,
                       int image_size) {
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

    for (const auto& obs : obstacles) {
        int cx = static_cast<int>((obs.x - min_x) * scale);
        int cy = image_size - static_cast<int>((obs.y - min_y) * scale);
        int radius = static_cast<int>(obs.radius * scale);

        cv::circle(g_original_image, cv::Point(cx, cy), radius, cv::Scalar(0, 0, 255), -1);
        cv::circle(g_original_image, cv::Point(cx, cy), radius, cv::Scalar(0, 0, 180), 2);
    }

    const auto& orig_path = vehicle.getOriginalPath();
    if (!orig_path.empty()) {
        for (size_t i = 1; i < orig_path.size(); i++) {
            int x1 = static_cast<int>((orig_path[i-1].x - min_x) * scale);
            int y1 = image_size - static_cast<int>((orig_path[i-1].y - min_y) * scale);
            int x2 = static_cast<int>((orig_path[i].x - min_x) * scale);
            int y2 = image_size - static_cast<int>((orig_path[i].y - min_y) * scale);

            cv::line(g_original_image, cv::Point(x1, y1), cv::Point(x2, y2),
                     cv::Scalar(255, 165, 0), 1, cv::LINE_AA);
        }
    }

    const auto& smooth_path = vehicle.getSmoothedPath();
    if (!smooth_path.empty()) {
        for (size_t i = 1; i < smooth_path.size(); i++) {
            int x1 = static_cast<int>((smooth_path[i-1].x - min_x) * scale);
            int y1 = image_size - static_cast<int>((smooth_path[i-1].y - min_y) * scale);
            int x2 = static_cast<int>((smooth_path[i].x - min_x) * scale);
            int y2 = image_size - static_cast<int>((smooth_path[i].y - min_y) * scale);

            cv::line(g_original_image, cv::Point(x1, y1), cv::Point(x2, y2),
                     cv::Scalar(0, 200, 255), 3, cv::LINE_AA);
        }
    }

    Point2D vehicle_pos = vehicle.getPosition();
    int vx = static_cast<int>((vehicle_pos.x - min_x) * scale);
    int vy = image_size - static_cast<int>((vehicle_pos.y - min_y) * scale);

    double yaw = vehicle.getYaw();
    int arrow_len = 25;
    int ax = vx + static_cast<int>(std::cos(yaw) * arrow_len);
    int ay = vy - static_cast<int>(std::sin(yaw) * arrow_len);

    cv::circle(g_original_image, cv::Point(vx, vy), 12, cv::Scalar(0, 255, 0), -1);
    cv::line(g_original_image, cv::Point(vx, vy), cv::Point(ax, ay),
             cv::Scalar(0, 200, 0), 4, cv::LINE_AA);

    int start_x = static_cast<int>((initial_start.x - min_x) * scale);
    int start_y = image_size - static_cast<int>((initial_start.y - min_y) * scale);
    cv::circle(g_original_image, cv::Point(start_x, start_y), 8, cv::Scalar(200, 200, 200), -1);

    int goal_x = static_cast<int>((goal.x - min_x) * scale);
    int goal_y = image_size - static_cast<int>((goal.y - min_y) * scale);
    cv::circle(g_original_image, cv::Point(goal_x, goal_y), 8, cv::Scalar(0, 0, 200), -1);

    char info[256];
    sprintf(info, "Vehicle: (%.1f, %.1f) Speed: %.1f m/s | Traveled: %.1fm | Path: %zu",
            vehicle_pos.x, vehicle_pos.y,
            vehicle.getCurrentSpeed(),
            vehicle.getDistanceTraveled(),
            vehicle.getPathIndex());
    cv::putText(g_original_image, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 100, 255), 1);

    cv::putText(g_original_image, "RRT Path Tracking Simulation", cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 100, 255), 2);

    int legend_x = 10;
    int legend_y = image_size - 100;

    cv::rectangle(g_original_image, cv::Point(legend_x, legend_y),
                  cv::Point(legend_x + 210, legend_y + 85),
                  cv::Scalar(255, 255, 255), -1);
    cv::rectangle(g_original_image, cv::Point(legend_x, legend_y),
                  cv::Point(legend_x + 210, legend_y + 85),
                  cv::Scalar(180, 180, 180), 1);

    cv::line(g_original_image, cv::Point(legend_x + 10, legend_y + 20),
             cv::Point(legend_x + 45, legend_y + 20),
             cv::Scalar(255, 165, 0), 1);
    cv::putText(g_original_image, "Original Path", cv::Point(legend_x + 50, legend_y + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    cv::line(g_original_image, cv::Point(legend_x + 10, legend_y + 42),
             cv::Point(legend_x + 45, legend_y + 42),
             cv::Scalar(0, 200, 255), 3);
    cv::putText(g_original_image, "Smoothed Path", cv::Point(legend_x + 50, legend_y + 47),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    cv::circle(g_original_image, cv::Point(legend_x + 27, legend_y + 66), 10, cv::Scalar(0, 255, 0), -1);
    cv::putText(g_original_image, "Vehicle", cv::Point(legend_x + 50, legend_y + 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    cv::putText(g_original_image, "按 ESC 退出", cv::Point(image_size - 150, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 100, 100), 1);

    g_display_image = g_original_image.clone();
}

std::string toJsonObstacles(const std::vector<AD::Planning::Obstacle>& obstacles) {
    std::stringstream ss;
    ss << "{\"obstacles\": [";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        ss << "{\"id\": " << i << ", \"x\": " << obstacles[i].x
           << ", \"y\": " << obstacles[i].y
           << ", \"z\": 0.0, \"radius\": " << obstacles[i].radius << "}";
        if (i < obstacles.size() - 1) ss << ",";
    }
    ss << "]}";
    return ss.str();
}

std::string toJsonPath(const std::vector<Point2D>& path) {
    std::stringstream ss;
    ss << "{\"path\": [";
    for (size_t i = 0; i < path.size(); ++i) {
        ss << "{\"x\": " << path[i].x
           << ", \"y\": " << path[i].y
           << ", \"z\": " << path[i].z << "}";
        if (i < path.size() - 1) ss << ",";
    }
    ss << "]}";
    return ss.str();
}

std::string toJsonVehicleState(const Point2D& pos, double yaw, double speed) {
    std::stringstream ss;
    ss << "{\"x\": " << pos.x << ", \"y\": " << pos.y << ", \"z\": " << pos.z
       << ", \"yaw\": " << yaw << ", \"speed\": " << speed << "}";
    return ss.str();
}

int main(int argc, char** argv) {
    std::cout << "=== RRT 路径跟踪模拟测试 ===" << std::endl;

    std::string mcap_filename = "../rrt_simulation.mcap";
    McapWriter mcap_writer;

    if (!mcap_writer.open(mcap_filename)) {
        std::cerr << "⚠️  Could not start MCAP writer, continuing without recording" << std::endl;
    } else {
        std::cout << "📝 MCAP recording enabled: " << mcap_filename << std::endl;
    }

    AD::FoxVisual::FoxgloveVisualizer visualizer;
    visualizer.init("0.0.0.0", 8765);
    visualizer.start();

    std::cout << "\n📊 Foxglove Visualization Server started on port 8765" << std::endl;
    std::cout << "📂 Open src/fox_visual/visualization.html in browser or load " << mcap_filename << " in Foxglove Studio\n" << std::endl;

    AD::Planning::RRTPlanner planner;
    planner.init(1.0, 0.5);

    double vehicle_length = 4.0;
    double vehicle_width = 1.8;
    double safety_distance = 0.5;

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
    obstacles.emplace_back(15.0, 10.0, 3.0);
    obstacles.emplace_back(30.0, 25.0, 2.5);
    obstacles.emplace_back(40.0, 35.0, 2.0);

    planner.setObstacles(obstacles);
    visualizer.publishObstacles(obstacles);

    auto smoother = AD::Planning::Smooth::SmootherFactory::create(
        AD::Planning::Smooth::SmootherType::BSPLINE);
    smoother->init(0.5, 0.5);

    VehicleSimulator vehicle;
    vehicle.setReplanDistance(10.0);

    AD::VehicleState current_start = start;
    AD::VehicleState current_goal = goal;

    std::cout << "\n=== 开始路径规划 ===" << std::endl;

    auto plan_start_time = std::chrono::high_resolution_clock::now();
    AD::Path path = planner.plan(current_start, current_goal);
    auto plan_end_time = std::chrono::high_resolution_clock::now();

    double plan_duration = std::chrono::duration<double, std::milli>(plan_end_time - plan_start_time).count();

    std::cout << "规划时间: " << plan_duration << " ms" << std::endl;
    std::cout << "路径点数: " << path.points.size() << std::endl;

    if (path.points.empty()) {
        std::cout << "路径规划失败！" << std::endl;
        return -1;
    }

    std::vector<Point2D> original_points;
    for (const auto& p : path.points) {
        original_points.push_back(Point2D(p.x, p.y, p.z));
    }

    AD::Path smoothed_path = smoother->smooth(path);

    std::vector<Point2D> smoothed_points;
    for (const auto& p : smoothed_path.points) {
        smoothed_points.push_back(Point2D(p.x, p.y, p.z));
    }

    vehicle.setPath(original_points, true);
    vehicle.setPath(smoothed_points, false);
    vehicle.init(Point2D(current_start.x, current_start.y, current_start.z), current_start.yaw);

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

    double padding = 15.0;
    min_x -= padding;
    max_x += padding;
    min_y -= padding;
    max_y += padding;

    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    int image_size = 700;
    double scale = std::min(image_size / range_x, image_size / range_y);

    std::cout << "\n=== 开始模拟 ===" << std::endl;
    std::cout << "车辆将以 5 m/s 的速度跟踪平滑路径" << std::endl;
    std::cout << "每行驶 10m 重规划一次" << std::endl;
    std::cout << "MCAP文件将直接保存到: " << mcap_filename << std::endl;
    std::cout << "按 ESC 退出模拟\n" << std::endl;

    visualizeSimulation(obstacles, vehicle, start, current_goal, scale, min_x, min_y, image_size);

    cv::namedWindow("RRT Path Tracking", cv::WINDOW_NORMAL);
    cv::resizeWindow("RRT Path Tracking", image_size, image_size);
    cv::setMouseCallback("RRT Path Tracking", onMouse, nullptr);

    auto last_time = std::chrono::high_resolution_clock::now();
    int iteration = 0;
    int replan_count = 0;

    g_simulation_running = true;

    std::thread visualization_thread([&]() {
        while (g_simulation_running) {
            AD::VehicleState v_state;
            v_state.x = vehicle.getPosition().x;
            v_state.y = vehicle.getPosition().y;
            v_state.z = vehicle.getPosition().z;
            v_state.yaw = vehicle.getYaw();
            visualizer.publishVehicleState(v_state);

            AD::Path fox_path;
            for (const auto& p : vehicle.getOriginalPath()) {
                fox_path.points.push_back({p.x, p.y, p.z});
            }
            visualizer.publishOriginalPath(fox_path);

            fox_path.points.clear();
            for (const auto& p : vehicle.getSmoothedPath()) {
                fox_path.points.push_back({p.x, p.y, p.z});
            }
            visualizer.publishSmoothedPath(fox_path);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    auto last_record_time = std::chrono::high_resolution_clock::now();
    int record_count = 0;

    while (g_simulation_running) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time).count();
        last_time = current_time;

        if (dt > 0.1) dt = 0.1;

        bool continue_tracking = vehicle.update(dt);

        Point2D v_pos = vehicle.getPosition();

        auto current_record_time = std::chrono::high_resolution_clock::now();
        auto record_elapsed = std::chrono::duration<double>(current_record_time - last_record_time).count();
        if (mcap_writer.isOpen() && record_elapsed >= 0.1) {
            last_record_time = current_record_time;

            mcap_writer.writeMessage("/planning/obstacles", toJsonObstacles(obstacles));
            mcap_writer.writeMessage("/planning/original_path", toJsonPath(vehicle.getOriginalPath()));
            mcap_writer.writeMessage("/planning/smoothed_path", toJsonPath(vehicle.getSmoothedPath()));
            mcap_writer.writeMessage("/vehicle/state",
                toJsonVehicleState(v_pos, vehicle.getYaw(), vehicle.getCurrentSpeed()));

            record_count += 4;
            if (record_count % 40 == 0) {
                std::cout << "\n📝 MCAP: " << record_count << " messages written\n";
            }
        }

        if (iteration % 20 == 0) {
            std::cout << "\rIteration: " << iteration
                      << " | Vehicle: (" << std::fixed << std::setprecision(1)
                      << v_pos.x << ", " << v_pos.y << ")"
                      << " | Speed: " << std::fixed << std::setprecision(1)
                      << vehicle.getCurrentSpeed() << " m/s"
                      << " | Traveled: " << std::fixed << std::setprecision(1)
                      << vehicle.getDistanceTraveled() << "m"
                      << " | Replan: " << replan_count
                      << " | Press ESC to exit";
            std::cout.flush();
        }

        if (vehicle.needsReplan()) {
            std::cout << "\n\n已行驶 " << std::fixed << std::setprecision(1)
                      << vehicle.getDistanceTraveled() << "m，重新规划..." << std::endl;

            replan_count++;

            current_start.x = v_pos.x;
            current_start.y = v_pos.y;
            current_start.yaw = vehicle.getYaw();

            auto new_plan_start = std::chrono::high_resolution_clock::now();
            AD::Path new_path = planner.plan(current_start, current_goal);
            auto new_plan_end = std::chrono::high_resolution_clock::now();

            if (!new_path.points.empty()) {
                std::vector<Point2D> new_original_points;
                for (const auto& p : new_path.points) {
                    new_original_points.push_back(Point2D(p.x, p.y, p.z));
                }

                AD::Path new_smoothed_path = smoother->smooth(new_path);
                std::vector<Point2D> new_smoothed_points;
                for (const auto& p : new_smoothed_path.points) {
                    new_smoothed_points.push_back(Point2D(p.x, p.y, p.z));
                }

                vehicle.setPath(new_original_points, true);
                vehicle.setPath(new_smoothed_points, false);
                vehicle.resetReplanDistance();

                double new_plan_duration = std::chrono::duration<double, std::milli>(
                    new_plan_end - new_plan_start).count();
                std::cout << "新路径规划完成，耗时: " << new_plan_duration << " ms" << std::endl;
                std::cout << "新路径点数: " << new_path.points.size() << std::endl;
            }
        }

        if (!continue_tracking) {
            std::cout << "\n\n到达目标点！准备下一个任务..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            break;
        }

        visualizeSimulation(obstacles, vehicle, start, current_goal, scale, min_x, min_y, image_size);

        cv::Mat show_img = g_display_image.clone();

        if (g_is_dragging && !g_selection_rect.empty()) {
            cv::rectangle(show_img, g_selection_rect, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("RRT Path Tracking", show_img);

        int key = cv::waitKey(30);

        if (key == 27) {
            std::cout << "\n\n用户退出模拟" << std::endl;
            g_simulation_running = false;
            break;
        } else if (key == 'r' || key == 'R') {
            resetView();
        }

        iteration++;
    }

    g_simulation_running = false;
    if (visualization_thread.joinable()) {
        visualization_thread.join();
    }

    cv::destroyAllWindows();

    std::cout << "\n\n=== 模拟统计 ===" << std::endl;
    std::cout << "总迭代次数: " << iteration << std::endl;
    std::cout << "重新规划次数: " << replan_count << std::endl;
    if (mcap_writer.isOpen()) {
        std::cout << "📝 MCAP messages written: " << record_count << std::endl;
    }

    mcap_writer.close();

    visualizer.stop();
    std::cout << "\n模拟完成！" << std::endl;
    std::cout << "📁 MCAP file saved: " << mcap_filename << std::endl;
    std::cout << "💡 Open this file in Foxglove Studio to view the recording\n" << std::endl;
    return 0;
}
