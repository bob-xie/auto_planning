# Foxglove Studio 可视化模块使用说明

## 概述

本模块为 RRT 路径规划算法提供了与 Foxglove Studio 集成的可视化功能，支持障碍物、原始路径和平滑路径的实时展示。

## 文件结构

```
ai/
├── include/fox_visual/
│   └── FoxgloveVisualizer.h          # 可视化器头文件
├── src/fox_visual/
│   ├── FoxgloveVisualizer.cpp        # 可视化器实现
│   ├── visualization.html            # 独立的 HTML 可视化界面
│   ├── README.md                     # 模块说明
│   └── USAGE.md                      # 使用说明（本文件）
└── test/planning/
    └── test_rrt.cpp                  # 集成了可视化的 RRT 测试程序
```

## 编译与运行

### 1. 编译项目

```bash
cd /home/a/brightdreamrobotics-master/ai/build
cmake ..
make -j4
```

### 2. 运行测试程序

```bash
cd /home/a/brightdreamrobotics-master/ai/build
./test/test_rrt
```

程序启动后会：
- 初始化 Foxglove 可视化服务器（端口 8765）
- 运行 RRT 路径规划算法
- 展示 OpenCV 可视化窗口
- 允许通过浏览器或 Foxglove Studio 访问数据

## 可视化方法

### 方法一：使用独立 HTML 界面（推荐用于快速测试）

1. 运行 test_rrt 程序
2. 在浏览器中打开文件：`/home/a/brightdreamrobotics-master/ai/src/fox_visual/visualization.html`
3. 点击 "Load Sample Data" 查看示例数据，或点击 "Fetch from Server" 获取实时数据

### 方法二：通过 HTTP API 直接访问数据

在浏览器中访问：`http://localhost:8765`

会返回 JSON 格式的数据，包含：
- obstacles: 障碍物列表
- original_path: 原始 RRT 路径
- smoothed_path: 平滑后的路径
- vehicle_state: 车辆状态

### 方法三：在 Foxglove Studio 中集成

1. 下载并安装 Foxglove Studio（https://foxglove.dev/）
2. 启动 Foxglove Studio
3. 添加 Custom Panel，使用 visualization.html 作为内容
4. 连接到本地服务器（localhost:8765）

## 数据格式

```json
{
  "obstacles": [
    {
      "id": 0,
      "x": 15.0,
      "y": 10.0,
      "z": 0.0,
      "radius": 3.0
    }
  ],
  "original_path": [
    {"x": 0.0, "y": 0.0, "z": 0.0},
    {"x": 5.0, "y": 3.0, "z": 0.0}
  ],
  "smoothed_path": [
    {"x": 0.0, "y": 0.0, "z": 0.0}
  ],
  "vehicle_state": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "yaw": 0.0
  }
}
```

## 功能特性

- ✅ 障碍物可视化（红色圆形）
- ✅ 原始路径展示（橙色线条）
- ✅ 平滑路径展示（蓝色线条）
- ✅ 起点/终点标记
- ✅ 网格背景
- ✅ 实时数据更新
- ✅ HTTP API 支持

## 测试建议

1. **连接测试**：确认浏览器可以访问 localhost:8765
2. **数据完整性**：验证返回的 JSON 数据包含所有字段
3. **更新频率**：多次运行测试程序，确认数据正确更新
4. **可视化展示**：使用 visualization.html 确认视觉效果符合预期

## 故障排除

### 端口被占用

如果端口 8765 被占用，修改 `test_rrt.cpp` 中的端口号：

```cpp
visualizer.init("0.0.0.0", 8766);  // 改为其他端口
```

### 无法连接

确保：
- 测试程序正在运行
- 防火墙允许本地连接
- 使用正确的端口号

## 扩展建议

- 添加 ROS 2 支持，通过 ROS 话题发送数据
- 实现 MCAP 文件记录功能
- 添加 3D 可视化支持
- 集成更多规划算法的可视化
- 添加时间序列数据记录

## 联系方式

如有问题，请查看项目根目录的 README 或联系开发团队。
