# Foxglove Studio 可视化使用说明

## 🎉 功能说明

现在支持两种方式进行可视化：

1. **WebSocket 实时可视化**（已有）
2. **MCAP 文件记录与回放**（新增）

---

## 📦 MCAP 文件记录（推荐用于回放）

### 快速开始

#### 1. 生成示例 MCAP 文件

如果不想先运行C++程序，可以直接使用Python脚本生成示例文件：

```bash
cd /home/a/brightdreamrobotics-master/ai
python3 mcap_writer.py --sample --output rrt_simulation.mcap
```

这会直接生成一个包含模拟数据的 MCAP 文件。

#### 2. 运行C++程序记录数据

```bash
cd /home/a/brightdreamrobotics-master/ai/build
./test/test_rrt
```

程序会：
- 运行车辆仿真
- 记录数据到 `../rrt_simulation.jsonl`（JSON Lines 格式）
- 按ESC退出程序

#### 3. 转换为 MCAP 格式

```bash
cd /home/a/brightdreamrobotics-master/ai
python3 mcap_writer.py --jsonl rrt_simulation.jsonl --output rrt_simulation.mcap
```

#### 4. 在 Foxglove Studio 中查看

1. 打开 Foxglove Studio
2. 点击 `File` -> `Open File`
3. 选择 `rrt_simulation.mcap`
4. 或直接拖拽文件到窗口中

---

## 📋 记录的话题

程序会记录以下话题的数据：

| 话题名称 | 数据内容 | 说明 |
|---------|---------|------|
| `/planning/obstacles` | 障碍物列表 | 包含位置和半径 |
| `/planning/original_path` | RRT原始路径 | 初始规划结果 |
| `/planning/smoothed_path` | 平滑后路径 | B样条平滑后的路径 |
| `/vehicle/state` | 车辆状态 | 位置、速度、朝向 |

---

## 🎯 完整流程示例

### 方案 A：先运行仿真再记录回放

```bash
# 终端1 - 运行C++程序
cd /home/a/brightdreamrobotics-master/ai/build
./test/test_rrt

# 让程序运行一段时间记录数据，然后按ESC退出

# 终端2 - 转换为MCAP
cd /home/a/brightdreamrobotics-master/ai
python3 mcap_writer.py --jsonl rrt_simulation.jsonl --output rrt_simulation.mcap

# 在Foxglove Studio中打开
# 1. Open Foxglove Studio
# 2. File -> Open File
# 3. 选择 rrt_simulation.mcap
```

### 方案 B：直接使用示例数据

```bash
cd /home/a/brightdreamrobotics-master/ai
python3 mcap_writer.py --sample
```

然后在Foxglove Studio中打开生成的文件。

---

## 🔧 Python依赖安装

如果需要安装mcap库：

```bash
pip3 install mcap
```

如果pip不可用，脚本会尝试自动安装。

---

## 📊 文件格式说明

### JSONL 格式（中间格式）

每行一条独立的JSON消息：

```json
{"topic": "/planning/obstacles", "data": {...}}
{"topic": "/vehicle/state", "data": {...}}
```

### MCAP 格式（最终格式）

二进制文件格式，包含时间戳、通道信息，可在Foxglove Studio中时间轴播放。

---

## 🎨 在 Foxglove Studio 中查看

### 查看JSON数据

1. 连接或打开文件后
2. 在左侧面板选择话题
3. 选择 "Raw Message" 或 "Plot" 视图

### 自定义可视化

1. 点击右上角 `+` 添加面板
2. 选择面板类型（如 "Plot", "Table", "Raw Message"）
3. 选择对应的话题
4. 选择要显示的字段

---

## 📁 文件位置

生成的文件默认在以下位置：

- `ai/rrt_simulation.jsonl` - JSONL数据文件
- `ai/rrt_simulation.mcap` - MCAP文件（最终）

---

## 🚀 车辆模拟功能更新

车辆现在支持：

- ✅ 平滑路径跟随
- ✅ 基于距离的重规划（每10米）
- ✅ 实时车辆状态更新
- ✅ 可视化显示当前位置

---

## 💡 提示

1. 如果Foxglove Studio无法识别话题，请确保消息格式正确
2. MCAP文件包含完整时间序列，支持暂停、倒退、慢速播放
3. 数据以10Hz记录（每0.1秒一条消息）

---

## 📝 版本更新

- **新功能**: MCAP文件记录与回放
- **改进**: 车辆路径跟随更平滑
- **新增**: 基于距离的重规划触发
