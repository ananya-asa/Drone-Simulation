# 🚁 Autonomous Drone Planner V5.4

> **Real-time LIDAR-based obstacle avoidance with dynamic A* path replanning for quadrotors.**

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Status](https://img.shields.io/badge/status-Active_Development-green.svg)
![Version](https://img.shields.io/badge/version-5.4-orange.svg)

---

## ✨ Features

* ✅ **Real-time LIDAR Integration:** Continuous point cloud processing (6000+ points/scan).
* ✅ **Dynamic A* Path Planning:** 26-directional movement, replans every 100ms.
* ✅ **Autonomous Obstacle Avoidance:** 0.4m safety margin, adapts to moving obstacles.
* ✅ **Robust Flight Control:** 20Hz command rate, altitude hold (±0.1m), failsafe detection.
* ✅ **Production-Grade Logging:** Comprehensive telemetry + CSV export.

### 📊 Performance Metrics

| Metric | Value | Status |
| :--- | :--- | :--- |
| **Mission Success Rate** | 100% | ✅ |
| **Waypoint Accuracy** | <0.3m | ✅ |
| **Path Planning Time** | 58ms (11 waypoints) | ✅ |
| **Obstacle Detection** | 0.4m safety margin | ✅ |
| **Altitude Stability** | ±0.1m | ✅ |
| **Control Frequency** | 20Hz | ✅ |

---

## 🏗️ System Architecture

```text
LIDAR Input (6000+ pts)
    ↓
[LIDAR Bridge] → ROS Topic
    ↓
[World Model] → Octree + Bounding Box
    ↓
[Path Planner] → A* Algorithm (26-dir)
    ↓
[Trajectory Follower] → Waypoint Generation
    ↓
[Main] → Flight Control (MAVSDK)
    ↓
Drone (PX4 OFFBOARD)

## 🛠️ Tech Stack

| Component | Technology | Version |
| :--- | :--- | :--- |
| **Flight Control** | MAVSDK | v3.10.2 |
| **Simulation** | Gazebo | 11+ |
| **Path Planning** | A\* Search | Custom C++17 |
| **Mapping** | Octree | AABB-based |
| **Point Cloud** | PCL | 1.11+ |
| **Math Library** | Eigen | 3.3+ |
| **Build System** | CMake | 3.10+ |

-----

## 🚀 Quick Start

### Prerequisites

```bash
# Ubuntu 20.04 LTS
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libpcl-dev \
    libeigen3-dev \
    ros-focal-mavsdk \
    gazebo11 \
    ros-focal-gazebo-* \
    python3-catkin-tools
```

### Build

```bash
cd ~/Drone-Simulation
mkdir build && cd build
cmake ..
make -j4
```

### Run in Simulation

**Terminal 1: Start PX4 + Gazebo**

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

**Terminal 2: Build and run autonomy brain**

```bash
cd ~/Drone-Simulation/build
./uav_planner
```

### Enter Test Goal

```plaintext
[INPUT] Enter goal position in NED coordinates:
  North (meters): 2
  East (meters): 0
  Down (meters, put -2.5 for 2.5m up): -2.5
```

### Expected Output

```plaintext
[FLIGHT] Taking off...
[LIDAR] Data received! Building map with 6185 points
[PLANNER] ✓ Path found in 58 iterations! (53 nodes expanded)
[ACT] Flying next 3 waypoints...
Waypoint 1/3: ... ✓ Reached
Waypoint 2/3: ... ✓ Reached
Waypoint 3/3: ... ✓ Reached
[GOAL REACHED]
[FLIGHT] Mission complete. Landing...
--- MISSION COMPLETE ---
```

-----

## 📁 Project Structure

```plaintext
Drone-Simulation/
├── README.md                   # This file
├── CMakeLists.txt
├── src/
│   ├── main.cpp                # Mission orchestration
│   ├── path_planner.cpp        # A* algorithm
│   ├── world_model.cpp         # Octree mapping
│   ├── trajectory_follower.cpp # Waypoint tracking
│   └── lidar_bridge.cpp        # LIDAR integration
├── include/
│   ├── path_planner.h
│   ├── world_model.h
│   ├── trajectory_follower.h
│   └── lidar_bridge.h
├── worlds/
│   └── default.sdf             # Gazebo world
├── docs/
│   ├── ARCHITECTURE.md         # System design
│   ├── SETUP.md                # Detailed setup
│   └── PERFORMANCE.md          # Benchmarks
└── build/
    └── uav_planner             # Compiled binary
```

-----

## 📈 Test Results

### Test 1: Static Obstacle (Box)

  * **Goal:** 2m North, 0m East, -2.5m Down
  * **Obstacle:** Box at center (1m × 1m × 1m)
  * **Result:** ✅ SUCCESS
  * **Timeline:**
      * Takeoff: 5s
      * Path Planning: 58ms (21 waypoints)
      * Flight Execution: 20s (dynamic replanning: 15 cycles)
      * Landing: 6s
      * **Total Mission Time:** \~35s (0 collisions)

### Test 2: Moving Obstacle (Animated Person)

  * **Goal:** 10m North, 0m East, -2.5m Down
  * **Obstacle:** Person moving left from center
  * **Result:** ✅ SUCCESS
  * **Key Finding:** Drone threads safe corridor (Y ≈ -0.1m) while person moves. Path adapts every cycle. Safety margin of 0.4m maintained.

### Test 3: Long-Distance Mission

  * **Goal:** 20m North, 0m East, -2.5m Down
  * **Result:** ✅ SUCCESS
  * **Stats:** Planning Time 150ms (92 waypoints), 30+ Replanning Cycles, Accuracy \<0.3m.

-----

## 🔧 Key Parameters (Tunable)

**Edit in `src/world_model.cpp`:**

```cpp
const float SAFETY_MARGIN = 0.4f;      // Distance from obstacles (meters)
const double RESOLUTION = 0.1;         // Grid resolution (meters)
const float INFLATION_RADIUS = 0.5f;   // Obstacle inflation
```

**Edit in `src/trajectory_follower.cpp`:**

```cpp
const float WAYPOINT_TOLERANCE = 0.3f; // Distance threshold (meters)
const int CMD_RATE_HZ = 20;            // Control frequency
```

-----

## 📊 Logging Output

Flight telemetry is automatically saved to `flight_log.csv`:

```csv
timestamp,x,y,z,vx,vy,vz,distance_to_goal
0.0,0.0,0.0,-2.5,0.0,0.0,0.0,10.0
1.5,0.1,0.05,-2.5,0.2,0.1,0.0,9.85
...
```

-----

## 🐛 Debugging

**Enable Debug Output:**
In `main.cpp`:

```cpp
#define DEBUG_MODE 1
```

**Common Status Messages:**

  * `[LIDAR_BRIDGE] Successfully subscribed` → LIDAR is working.
  * `[PLANNER] ✓ Path found` → A\* is working.
  * `[FLIGHT] Taking off...` → MAVSDK is communicating.

-----

## 🚀 Next Steps (Roadmap)

  - [ ] **Phase 2: Kalman Filter** - Fuse accelerometer + GPS data for \<0.1m accuracy.
  - [ ] **Phase 3: Minimum-Snap Trajectory** - Replace waypoints with smooth trajectory using quadratic programming (OSQP).
  - [ ] **Phase 4: MPC** - Implement nonlinear Model Predictive Control for aggressive flight.
  - [ ] **Phase 5: GPU Acceleration** - Port A\* to CUDA for 10x speed improvement.

-----

## 📚 References & Resources

  * MAVSDK Documentation
  * PX4 Autopilot
  * **Books:**
      * *Probabilistic Robotics* (Thrun et al.)
      * *Robotics, Vision, and Control* (Corke)

-----

## 👤 Author

**Ananya**

  * Full-Stack Developer & Robotics Enthusiast 
  * GitHub: @ananya-asa

## 📄 License

MIT License - See LICENSE file for details.

```
```
