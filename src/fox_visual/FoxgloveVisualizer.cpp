#include "fox_visual/FoxgloveVisualizer.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace AD {
namespace FoxVisual {

FoxgloveVisualizer::FoxgloveVisualizer()
    : host_("0.0.0.0")
    , port_(8765)
    , running_(false) {
}

FoxgloveVisualizer::~FoxgloveVisualizer() {
    stop();
}

bool FoxgloveVisualizer::init(const std::string& host, int port) {
    host_ = host;
    port_ = port;
    std::cout << "Foxglove Visualizer initialized on " << host_ << ":" << port_ << std::endl;
    return true;
}

void FoxgloveVisualizer::start() {
    if (running_) {
        std::cout << "Foxglove Visualizer already running" << std::endl;
        return;
    }
    running_ = true;
    server_thread_ = std::thread(&FoxgloveVisualizer::serverLoop, this);
    std::cout << "Foxglove Visualizer started" << std::endl;
}

void FoxgloveVisualizer::stop() {
    running_ = false;
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    std::cout << "Foxglove Visualizer stopped" << std::endl;
}

void FoxgloveVisualizer::publishObstacles(const std::vector<Planning::Obstacle>& obstacles) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_obstacles_ = obstacles;
}

void FoxgloveVisualizer::publishOriginalPath(const Path& path) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_original_path_ = path;
}

void FoxgloveVisualizer::publishSmoothedPath(const Path& path) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_smoothed_path_ = path;
}

void FoxgloveVisualizer::publishVehicleState(const VehicleState& state) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_vehicle_state_ = state;
}

std::string base64_encode(const std::string& input) {
    const std::string base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    
    for (size_t idx = 0; idx < input.size(); idx++) {
        char_array_3[i++] = input[idx];
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            
            for (i = 0; i < 4; i++) {
                ret += base64_chars[char_array_4[i]];
            }
            i = 0;
        }
    }
    
    if (i) {
        for (j = i; j < 3; j++) {
            char_array_3[j] = '\0';
        }
        
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        
        for (j = 0; j < i + 1; j++) {
            ret += base64_chars[char_array_4[j]];
        }
        
        while (i++ < 3) {
            ret += '=';
        }
    }
    
    return ret;
}

void compute_sha1(const unsigned char* data, size_t len, unsigned char* digest) {
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
    
    while (len >= 64) {
        uint32_t w[16];
        for (int i = 0; i < 16; i++) {
            w[i] = (data[i * 4] << 24) | (data[i * 4 + 1] << 16) | (data[i * 4 + 2] << 8) | data[i * 4 + 3];
        }
        
        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        
        auto f = [](uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (~x & z); };
        auto g = [](uint32_t x, uint32_t y, uint32_t z) { return (x & z) ^ (y & ~z); };
        auto h = [](uint32_t x, uint32_t y, uint32_t z) { return x ^ y ^ z; };
        auto i = [](uint32_t x, uint32_t y, uint32_t z) { return y ^ (x | ~z); };
        auto rotl = [](uint32_t x, int n) { return (x << n) | (x >> (32 - n)); };
        
        uint32_t k[] = {
            0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999,
            0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1,
            0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC,
            0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6
        };
        
        for (int j = 0; j < 64; j++) {
            uint32_t temp;
            if (j < 16) {
                temp = rotl(a, 5) + f(b, c, d) + e + k[j] + w[j];
            } else if (j < 32) {
                temp = rotl(a, 5) + g(b, c, d) + e + k[j] + w[(5*j + 1) % 16];
            } else if (j < 48) {
                temp = rotl(a, 5) + h(b, c, d) + e + k[j] + w[(3*j + 5) % 16];
            } else {
                temp = rotl(a, 5) + i(b, c, d) + e + k[j] + w[(7*j) % 16];
            }
            e = d; d = c; c = rotl(b, 30); b = a; a = temp;
        }
        
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
        data += 64; len -= 64;
    }
    
    uint32_t w[16] = {0};
    for (size_t i = 0; i < len; i++) {
        w[i >> 2] |= (data[i] << ((3 - (i & 3)) * 8));
    }
    w[len >> 2] |= 0x80 << ((3 - (len & 3)) * 8);
    w[15] = len * 8;
    
    uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
    
    auto f = [](uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (~x & z); };
    auto g = [](uint32_t x, uint32_t y, uint32_t z) { return (x & z) ^ (y & ~z); };
    auto h = [](uint32_t x, uint32_t y, uint32_t z) { return x ^ y ^ z; };
    auto i = [](uint32_t x, uint32_t y, uint32_t z) { return y ^ (x | ~z); };
    auto rotl = [](uint32_t x, int n) { return (x << n) | (x >> (32 - n)); };
    
    uint32_t k[] = {
        0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999, 0x5A827999,
        0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1, 0x6ED9EBA1,
        0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC, 0x8F1BBCDC,
        0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6, 0xCA62C1D6
    };
    
    for (int j = 0; j < 64; j++) {
        uint32_t temp;
        if (j < 16) {
            temp = rotl(a, 5) + f(b, c, d) + e + k[j] + w[j];
        } else if (j < 32) {
            temp = rotl(a, 5) + g(b, c, d) + e + k[j] + w[(5*j + 1) % 16];
        } else if (j < 48) {
            temp = rotl(a, 5) + h(b, c, d) + e + k[j] + w[(3*j + 5) % 16];
        } else {
            temp = rotl(a, 5) + i(b, c, d) + e + k[j] + w[(7*j) % 16];
        }
        e = d; d = c; c = rotl(b, 30); b = a; a = temp;
    }
    
    for (int i = 0; i < 4; i++) {
        digest[i] = (h0 >> (24 - i * 8)) & 0xff;
        digest[i + 4] = (h1 >> (24 - i * 8)) & 0xff;
        digest[i + 8] = (h2 >> (24 - i * 8)) & 0xff;
        digest[i + 12] = (h3 >> (24 - i * 8)) & 0xff;
        digest[i + 16] = (h4 >> (24 - i * 8)) & 0xff;
    }
}

std::string generateWebSocketKey(const std::string& client_key) {
    std::string magic_string = client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    
    unsigned char digest[20];
    compute_sha1(reinterpret_cast<const unsigned char*>(magic_string.c_str()), magic_string.size(), digest);
    
    return base64_encode(std::string(reinterpret_cast<char*>(digest), 20));
}

bool performWebSocketHandshake(int client_socket) {
    char buffer[4096] = {0};
    ssize_t bytes_read = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    
    if (bytes_read <= 0) {
        return false;
    }
    
    std::string request(buffer, bytes_read);
    
    size_t key_pos = request.find("Sec-WebSocket-Key: ");
    if (key_pos == std::string::npos) {
        return false;
    }
    
    key_pos += 19;
    size_t key_end = request.find("\r\n", key_pos);
    std::string client_key = request.substr(key_pos, key_end - key_pos);
    
    std::string response_key = generateWebSocketKey(client_key);
    
    std::string response = 
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + response_key + "\r\n"
        "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
        "\r\n";
    
    send(client_socket, response.c_str(), response.length(), 0);
    return true;
}

void sendWebSocketMessage(int client_socket, const std::string& data) {
    size_t payload_len = data.length();
    std::vector<uint8_t> frame;
    
    frame.push_back(0x81);
    
    if (payload_len <= 125) {
        frame.push_back(static_cast<uint8_t>(payload_len));
    } else if (payload_len <= 65535) {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xff));
        frame.push_back(static_cast<uint8_t>(payload_len & 0xff));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; i--) {
            frame.push_back(static_cast<uint8_t>((payload_len >> (i * 8)) & 0xff));
        }
    }
    
    frame.insert(frame.end(), data.begin(), data.end());
    
    send(client_socket, reinterpret_cast<const char*>(frame.data()), frame.size(), 0);
}

void FoxgloveVisualizer::serverLoop() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        close(server_fd);
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        return;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen");
        close(server_fd);
        return;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  Foxglove Studio WebSocket Server" << std::endl;
    std::cout << "  Protocol: foxglove.websocket.v1" << std::endl;
    std::cout << "  Listening on port " << port_ << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\n📡 Waiting for Foxglove Studio connection..." << std::endl;
    std::cout << "   Connect using: ws://localhost:" << port_ << std::endl;
    std::cout << "\n📋 Available topics:" << std::endl;
    std::cout << "   • /scene                  - 3D Scene visualization" << std::endl;
    std::cout << "   • /planning/obstacles     - Obstacles (position + radius)" << std::endl;
    std::cout << "   • /planning/original_path - Original RRT path" << std::endl;
    std::cout << "   • /planning/smoothed_path - Smoothed path" << std::endl;
    std::cout << "   • /vehicle/state          - Vehicle state" << std::endl;
    std::cout << std::endl;

    while (running_) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(server_fd, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int activity = select(server_fd + 1, &read_fds, NULL, NULL, &timeout);

        if (activity < 0) {
            continue;
        }

        if (FD_ISSET(server_fd, &read_fds)) {
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
                perror("accept");
                continue;
            }

            std::cout << "\n🔄 New connection from " << inet_ntoa(address.sin_addr) << std::endl;
            std::cout << "   Performing WebSocket handshake..." << std::endl;
            
            if (!performWebSocketHandshake(new_socket)) {
                std::cout << "❌ Handshake failed" << std::endl;
                close(new_socket);
                continue;
            }

            std::cout << "✅ WebSocket handshake successful!" << std::endl;
            std::cout << "   Protocol: foxglove.websocket.v1" << std::endl;
            
            std::cout << "   Sending serverInfo..." << std::endl;
            std::string server_info = 
                "{\"op\":\"serverInfo\",\"name\":\"RRT Path Planning\","
                "\"capabilities\":[\"json\",\"image\"],"
                "\"supportedEncodings\":[\"json\",\"protobuf\"],"
                "\"metadata\":{}}";
            sendWebSocketMessage(new_socket, server_info);

            std::cout << "   Sending advertise messages..." << std::endl;
            sendWebSocketMessage(new_socket, 
                "{\"op\":\"advertise\",\"topics\":["
                "{\"name\":\"/scene\",\"encoding\":\"json\",\"schemaName\":\"foxglove.SceneUpdate\"},"
                "{\"name\":\"/planning/obstacles\",\"encoding\":\"json\",\"schemaName\":\"Obstacles\"},"
                "{\"name\":\"/planning/original_path\",\"encoding\":\"json\",\"schemaName\":\"Path\"},"
                "{\"name\":\"/planning/smoothed_path\",\"encoding\":\"json\",\"schemaName\":\"Path\"},"
                "{\"name\":\"/vehicle/state\",\"encoding\":\"json\",\"schemaName\":\"VehicleState\"}"
                "]}");

            std::cout << "   Starting continuous data stream at 10 Hz..." << std::endl;

            int message_count = 0;
            int advertise_count = 0;
            auto last_send_time = std::chrono::steady_clock::now();

            while (running_) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_send_time).count();

                if (elapsed >= 100) {
                    last_send_time = current_time;

                    std::lock_guard<std::mutex> lock(data_mutex_);

                    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();

                    std::stringstream ss;
                    ss << "{\"op\":\"publish\",\"topic\":\"/scene\",\"timestamp\":" << timestamp 
                       << ",\"msg\":{\"entities\":[";

                    for (size_t i = 0; i < current_obstacles_.size(); ++i) {
                        ss << "{\"id\":\"obstacle_" << i << "\","
                           << "\"frame_id\":\"map\","
                           << "\"position\":{\"x\":" << std::fixed << std::setprecision(3) << current_obstacles_[i].x 
                           << ",\"y\":" << current_obstacles_[i].y 
                           << ",\"z\":0.0},"
                           << "\"color\":{\"r\":1.0,\"g\":0.0,\"b\":0.0,\"a\":0.8},"
                           << "\"primitive\":{\"sphere\":{\"radius\":" << current_obstacles_[i].radius << "}}}";
                        if (i < current_obstacles_.size() - 1) ss << ",";
                    }

                    if (!current_obstacles_.empty() && (!current_original_path_.points.empty() || !current_smoothed_path_.points.empty())) {
                        ss << ",";
                    }

                    if (!current_original_path_.points.empty()) {
                        ss << "{\"id\":\"original_path\","
                           << "\"frame_id\":\"map\","
                           << "\"color\":{\"r\":1.0,\"g\":0.647,\"b\":0.0,\"a\":1.0},"
                           << "\"primitive\":{\"line_strip\":{\"points\":[";
                        for (size_t i = 0; i < current_original_path_.points.size(); ++i) {
                            ss << "{\"x\":" << std::fixed << std::setprecision(3) << current_original_path_.points[i].x 
                               << ",\"y\":" << current_original_path_.points[i].y 
                               << ",\"z\":" << current_original_path_.points[i].z << "}";
                            if (i < current_original_path_.points.size() - 1) ss << ",";
                        }
                        ss << "]}}}";
                        
                        if (!current_smoothed_path_.points.empty()) {
                            ss << ",";
                        }
                    }

                    if (!current_smoothed_path_.points.empty()) {
                        ss << "{\"id\":\"smoothed_path\","
                           << "\"frame_id\":\"map\","
                           << "\"color\":{\"r\":0.0,\"g\":0.8,\"b\":1.0,\"a\":1.0},"
                           << "\"primitive\":{\"line_strip\":{\"points\":[";
                        for (size_t i = 0; i < current_smoothed_path_.points.size(); ++i) {
                            ss << "{\"x\":" << std::fixed << std::setprecision(3) << current_smoothed_path_.points[i].x 
                               << ",\"y\":" << current_smoothed_path_.points[i].y 
                               << ",\"z\":" << current_smoothed_path_.points[i].z << "}";
                            if (i < current_smoothed_path_.points.size() - 1) ss << ",";
                        }
                        ss << "]}}}";
                    }

                    double yaw = current_vehicle_state_.yaw;
                    double cos_yaw = cos(yaw / 2);
                    double sin_yaw = sin(yaw / 2);
                    
                    ss << ",{\"id\":\"vehicle_body\","
                       << "\"frame_id\":\"map\","
                       << "\"position\":{\"x\":" << std::fixed << std::setprecision(3) << current_vehicle_state_.x 
                       << ",\"y\":" << current_vehicle_state_.y 
                       << ",\"z\":0.75},"
                       << "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":" << sin_yaw << ",\"w\":" << cos_yaw << "},"
                       << "\"color\":{\"r\":0.0,\"g\":1.0,\"b\":0.0,\"a\":1.0},"
                       << "\"primitive\":{\"box\":{\"dimensions\":[4.0,2.0,1.5]}}}";

                    ss << ",{\"id\":\"vehicle_direction\","
                       << "\"frame_id\":\"map\","
                       << "\"position\":{\"x\":" << std::fixed << std::setprecision(3) << current_vehicle_state_.x 
                       << ",\"y\":" << current_vehicle_state_.y 
                       << ",\"z\":0.75},"
                       << "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":" << sin_yaw << ",\"w\":" << cos_yaw << "},"
                       << "\"color\":{\"r\":0.0,\"g\":1.0,\"b\":0.0,\"a\":1.0},"
                       << "\"primitive\":{\"arrow\":{\"shaft_length\":3.0,\"head_length\":1.0,\"head_radius\":0.5}}}";

                    ss << "],\"deleted_entity_ids\":[]}}";
                    sendWebSocketMessage(new_socket, ss.str());
                    message_count++;

                    ss.str("");
                    ss << "{\"op\":\"publish\",\"topic\":\"/planning/obstacles\",\"timestamp\":" << timestamp 
                       << ",\"msg\":{\"obstacles\":[";
                    for (size_t i = 0; i < current_obstacles_.size(); ++i) {
                        ss << "{\"id\":" << i 
                           << ",\"x\":" << std::fixed << std::setprecision(3) << current_obstacles_[i].x 
                           << ",\"y\":" << current_obstacles_[i].y 
                           << ",\"z\":0.0"
                           << ",\"radius\":" << current_obstacles_[i].radius << "}";
                        if (i < current_obstacles_.size() - 1) ss << ",";
                    }
                    ss << "]}}";
                    sendWebSocketMessage(new_socket, ss.str());
                    message_count++;

                    ss.str("");
                    ss << "{\"op\":\"publish\",\"topic\":\"/planning/original_path\",\"timestamp\":" << timestamp 
                       << ",\"msg\":{\"path\":[";
                    for (size_t i = 0; i < current_original_path_.points.size(); ++i) {
                        ss << "{\"x\":" << std::fixed << std::setprecision(3) << current_original_path_.points[i].x 
                           << ",\"y\":" << current_original_path_.points[i].y 
                           << ",\"z\":" << current_original_path_.points[i].z << "}";
                        if (i < current_original_path_.points.size() - 1) ss << ",";
                    }
                    ss << "]}}";
                    sendWebSocketMessage(new_socket, ss.str());
                    message_count++;

                    ss.str("");
                    ss << "{\"op\":\"publish\",\"topic\":\"/planning/smoothed_path\",\"timestamp\":" << timestamp 
                       << ",\"msg\":{\"path\":[";
                    for (size_t i = 0; i < current_smoothed_path_.points.size(); ++i) {
                        ss << "{\"x\":" << std::fixed << std::setprecision(3) << current_smoothed_path_.points[i].x 
                           << ",\"y\":" << current_smoothed_path_.points[i].y 
                           << ",\"z\":" << current_smoothed_path_.points[i].z << "}";
                        if (i < current_smoothed_path_.points.size() - 1) ss << ",";
                    }
                    ss << "]}}";
                    sendWebSocketMessage(new_socket, ss.str());
                    message_count++;

                    ss.str("");
                    ss << "{\"op\":\"publish\",\"topic\":\"/vehicle/state\",\"timestamp\":" << timestamp 
                       << ",\"msg\":{"
                       << "\"x\":" << current_vehicle_state_.x 
                       << ",\"y\":" << current_vehicle_state_.y 
                       << ",\"z\":" << current_vehicle_state_.z 
                       << ",\"yaw\":" << current_vehicle_state_.yaw 
                       << "}}";
                    sendWebSocketMessage(new_socket, ss.str());
                    message_count++;

                    advertise_count++;
                    if (advertise_count % 50 == 0) {
                        std::cout << "   📊 Sent " << message_count << " messages (" 
                                  << advertise_count << " sets of 5 topics)" << std::endl;
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            close(new_socket);
            std::cout << "\n❌ Client disconnected" << std::endl;
            std::cout << "   Total messages sent: " << message_count << std::endl;
            std::cout << "\n📡 Waiting for new connection..." << std::endl;
        }
    }

    close(server_fd);
}

}
}