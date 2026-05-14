# Foxglove Studio Visualization Module

本模块用于将RRT规划算法的结果可视化到Foxglove Studio中。

## 功能

- 障碍物可视化
- 原始规划路径显示
- 平滑后路径显示

## 使用方法

### 方式1: 通过HTTP API获取JSON数据（已实现）

1. 运行test_rrt程序
2. 在浏览器或Foxglove Studio中访问 http://localhost:8765 查看数据
3. 使用下面的可视化界面展示数据

### 方式2: Foxglove Studio自定义面板（推荐）

将visualization.html文件作为Custom Panel添加到Foxglove Studio中。

## 文件结构

```
include/fox_visual/
    FoxgloveVisualizer.h     - 可视化器头文件
src/fox_visual/
    FoxgloveVisualizer.cpp   - 可视化器实现
    visualization.html       - 用于Foxglove Studio的可视化界面
    README.md                - 本说明文件
```

## 数据格式

JSON数据格式：
```json
{
  "obstacles": [
    {"id": 0, "x": 15.0, "y": 10.0, "z": 0.0, "radius": 3.0}
  ],
  "original_path": [
    {"x": 0.0, "y": 0.0, "z": 0.0}
  ],
  "smoothed_path": [
    {"x": 0.0, "y": 0.0, "z": 0.0}
  ],
  "vehicle_state": {
    "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0
  }
}
```
