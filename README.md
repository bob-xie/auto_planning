# Autonomous Driving Simulation Platform

A comprehensive autonomous driving simulation platform featuring multiple path planning algorithms, vehicle dynamics simulation, real-time visualization, and data recording capabilities.

## 🏗️ Architecture Overview

```
ai/
├── include/                    # Header files
│   ├── app/                   # Application management
│   ├── common/                # Common data structures
│   ├── communication/         # Communication interfaces
│   ├── control/               # Control algorithms
│   ├── fox_visual/            # Foxglove visualization
│   ├── localization/          # Localization systems
│   ├── perception/            # Perception modules
│   ├── planning/               # Path planning algorithms
│   │   └── smooth/            # Path smoothing algorithms
│   ├── safety/                # Safety management
│   └── vehicle/               # Vehicle models
├── src/                       # Source code implementation
│   ├── app/
│   ├── common/
│   ├── communication/
│   ├── control/
│   ├── fox_visual/
│   ├── localization/
│   ├── perception/
│   ├── planning/
│   │   └── smooth/
│   ├── safety/
│   └── vehicle/
├── test/                      # Test programs
│   └── planning/
├── config/                    # Configuration files
├── proto/                     # Protocol Buffer definitions
├── third_party/               # Third-party libraries (OSQP)
└── build/                     # Build directory
```

## 🚗 Core Modules

### 1. Vehicle Models (`src/vehicle/`)

The platform supports multiple vehicle models:

- **SUVVehicle**: Sport Utility Vehicle model (4.5m × 1.8m × 1.6m)
- **SedanVehicle**: Sedan model (4.8m × 1.8m × 1.4m)

Vehicle parameters include:
- Maximum speed: 5.0 m/s
- Maximum acceleration: 1.0 m/s²
- Maximum steering angle: 30°
- Wheelbase: 2.8m

### 2. Path Planning Algorithms (`src/planning/`)

#### RRT (Rapidly-exploring Random Tree)
- **File**: `RRTPlanner.h/cpp`
- **Features**:
  - Configurable step size and goal tolerance
  - Obstacle collision checking
  - Bidirectional tree growth
- **Usage**:
  ```bash
  ./build/test/test_rrt
  ```

#### RRT* (Optimized RRT)
- **File**: `RRTPlanner.h/cpp` (with optimization)
- **Features**:
  - Asymptotically optimal path
  - Rewiring mechanism
  - Minimum path cost

#### A* Algorithm
- **File**: `AStarPlanner.h/cpp`
- **Features**:
  - Grid-based planning
  - Heuristic search
  - Optimal path guarantee

#### DWA (Dynamic Window Approach)
- **File**: `DWAPlanner.h/cpp`
- **Features**:
  - Velocity sampling
  - Trajectory evaluation
  - Real-time obstacle avoidance
- **Usage**:
  ```bash
  ./build/test/test_dwa
  ```

#### Lattice Planner
- **File**: `LatticePlanner.h/cpp`
- **Features**:
  - State lattice planning
  - Curvature-continuous paths
- **Usage**:
  ```bash
  ./build/test/test_lattice
  ```

### 3. Path Smoothing (`src/planning/smooth/`)

#### B-Spline Smoothing
- **File**: `bspline/BsplineSmoother.h/cpp`
- **Features**:
  - Uniform B-spline interpolation
  - Configurable control points
  - Smooth curvature

#### Window Smoothing
- **File**: `window/WindowSmoother.h/cpp`
- **Features**:
  - Moving window optimization
  - Low computational cost

### 4. Localization (`src/localization/`)

- **EKF (Extended Kalman Filter)**: Non-linear state estimation
- **IMU**: Inertial measurement unit integration
- **RTKGPS**: Real-Time Kinematic GPS integration
- **FusionLocalizer**: Multi-sensor fusion

### 5. Control (`src/control/`)

- **LongitudinalController**: Speed control
- **LateralController**: Steering control
- **MPCController**: Model Predictive Control
- **ControllerBase**: Abstract base class for controllers

### 6. Safety (`src/safety/`)

- **SafetyManager**: Safety monitoring and emergency stop
- Collision detection
- Boundary checking
- Emergency brake trigger

## 🔄 Communication

### WebSocket Server
- **Port**: 8765
- **Protocol**: foxglove.websocket.v1
- **Features**:
  - Real-time data streaming
  - Multiple topic support
  - JSON message encoding

### Topics Available
| Topic | Type | Description |
|-------|------|-------------|
| `/planning/obstacles` | Obstacles | List of detected obstacles |
| `/planning/original_path` | Path | Raw RRT/A* planned path |
| `/planning/smoothed_path` | Path | Smoothed trajectory |
| `/vehicle/state` | VehicleState | Current vehicle pose and speed |
| `/scene` | foxglove.SceneUpdate | 3D scene for visualization |
| `/tf` | tf2_msgs/TFMessage | Coordinate frame transforms |

## 🎮 Visualization

### Foxglove Studio Integration

The platform provides comprehensive visualization through Foxglove Studio:

#### Method 1: Real-time WebSocket Connection

```bash
# Run the simulation
cd build
./test/test_rrt

# In Foxglove Studio
# 1. Click "Open Connection" → "WebSocket"
# 2. Connect to: ws://localhost:8765
# 3. Add "3D" panel and select topics
```

#### Method 2: MCAP File Recording

```bash
# The program automatically records to MCAP file
# Located at: ../rrt_simulation.mcap

# Or generate a sample file
python3 mcap_writer_helper.py sample.mcap --sample
```

### 3D Visualization Elements

| Element | Color | Shape | Description |
|---------|-------|-------|-------------|
| Obstacles | Red | Sphere | Circular obstacles with radius |
| Original Path | Orange | Line Strip | RRT raw path |
| Smoothed Path | Cyan | Line Strip | B-spline smoothed path |
| Vehicle Body | Green | Box (4×2×1.5m) | Current vehicle position |
| Vehicle Direction | Green | Arrow | Vehicle heading direction |

### Configuration Files

- **Foxglove Settings**: `src/fox_visual/README.md`
- **Visualization HTML**: `src/fox_visual/visualization.html`

## 📦 Dependencies

### System Dependencies
- CMake >= 3.10
- GCC/G++ with C++14 support
- OpenCV >= 4.0
- Eigen3
- PCL (Point Cloud Library)
- Python3

### Python Dependencies
```bash
pip install mcap
```

### Build Instructions

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Compile
make -j4

# Run tests
./test/test_rrt        # RRT path planning
./test/test_dwa       # DWA planning
./test/test_lattice   # Lattice planning
./test/test_astar     # A* planning
```

## 🧪 Testing

### RRT Path Planning Test
```bash
./test/test_rrt
```
Features:
- Vehicle speed: 5 m/s
- Replanning distance: 10m
- Real-time visualization
- MCAP recording

### DWA Test
```bash
./test/test_dwa
```
Features:
- Dynamic window approach
- Velocity sampling
- Trajectory evaluation

## 📊 Data Structures

### Path
```cpp
struct Point3D {
    double x, y, z;
};

struct Path {
    std::vector<Point3D> points;
    double cost;
};
```

### VehicleState
```cpp
struct VehicleState {
    double x, y, z;      // Position
    double yaw;          // Heading angle
    double speed;        // Velocity
    double acceleration; // Acceleration
};
```

### Obstacle
```cpp
struct Obstacle {
    double x, y, z;      // Center position
    double radius;       // Collision radius
    double velocity_x;   // Velocity in X
    double velocity_y;   // Velocity in Y
};
```

## 🔧 Configuration

### Vehicle Configuration (`config/vehicles/`)
- `suv.yaml`: SUV vehicle parameters
- `sedan.yaml`: Sedan vehicle parameters

Example:
```yaml
vehicle:
  length: 4.5
  width: 1.8
  height: 1.6
  wheelbase: 2.8
  max_speed: 5.0
  max_acceleration: 1.0
  max_steering_angle: 0.524  # 30 degrees
```

## 📝 Recording and Playback

### MCAP File Format

The platform records simulation data in MCAP format for later analysis:

```bash
# Recording is automatic when running tests
# Output: ../rrt_simulation.mcap

# View in Foxglove Studio
# 1. Open Foxglove Studio
# 2. File → Open File → Select .mcap file
# 3. Add desired panels for visualization
```

### Recorded Topics
- All planning topics at 10 Hz
- Vehicle state at 10 Hz
- Scene updates for 3D replay

## 📚 Documentation

- `src/fox_visual/README.md` - Visualization module documentation
- `src/fox_visual/USAGE.md` - Detailed usage guide
- `FOXGLOVE_README.md` - Foxglove integration guide

## 🎯 Usage Example

```bash
# Complete workflow
cd build
cmake .. && make -j4

# Run RRT simulation
./test/test_rrt

# In another terminal, open Foxglove Studio
# Connect to ws://localhost:8765 for real-time visualization

# Or open the recorded MCAP file in Foxglove Studio
# File: ai/rrt_simulation.mcap
```

## ⚙️ Architecture Patterns

### Design Patterns Used

1. **Factory Pattern**: `VehicleFactory`, `SmootherFactory`
2. **Strategy Pattern**: Different planners interchangeable
3. **Observer Pattern**: Real-time data publishing
4. **Singleton Pattern**: Configuration management

### Code Organization

- **Modularity**: Each module is self-contained
- **Header-only where appropriate**: For simple utilities
- **Shared libraries**: For complex algorithms
- **Clean separation**: Between interface and implementation

## 🚨 Safety Features

1. **Collision Detection**: Real-time obstacle checking
2. **Boundary Enforcement**: Workspace limits
3. **Emergency Stop**: Manual shutdown capability
4. **State Validation**: Input parameter checking

## 🔮 Future Enhancements

- [ ] ROS2 integration
- [ ] LiDAR-based perception
- [ ] Camera-based object detection
- [ ] Multi-vehicle coordination
- [ ] Hardware-in-the-loop testing

## 📄 License

This project is proprietary software for Bright Dream Robotics.

## 🤝 Contact

**Bright Dream Robotics**
- Website: www.brightdreamrobotics.com
- Email: contact@brightdreamrobotics.com

## 🗺️ Roadmap

### Phase 1 ✅ - Completed
- [x] Core path planning algorithms
- [x] Vehicle simulation
- [x] Basic visualization

### Phase 2 ✅ - Completed
- [x] Foxglove integration
- [x] MCAP recording
- [x] Multi-algorithm support

### Phase 3 🔄 - In Progress
- [x] Real-time WebSocket streaming
- [x] 3D scene visualization
- [ ] Enhanced perception integration
- [ ] Multi-vehicle scenarios
