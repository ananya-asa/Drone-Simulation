
````markdown
# 🛠️ Setup & Installation Guide

## System Requirements

### Hardware
* **Minimum:** Laptop/Desktop with 8GB RAM, Intel i5/AMD Ryzen 5
* **Recommended:** 16GB RAM, i7/Ryzen 7 (for faster simulation)
* **GPU (Optional):** CUDA-capable GPU for acceleration

### Software
* **OS:** Ubuntu 20.04 LTS (Focal Fossa) **[REQUIRED]**
* **ROS:** ROS Noetic or ROS 2 Foxy
* **Gazebo:** 11.0+
* **CMake:** 3.10+
* **C++:** C++17 compiler (g++ 7+)

---

## Step 1: Install Ubuntu 20.04

If not already installed, download and install Ubuntu 20.04:
* **Download:** [Ubuntu 20.04 Desktop](https://ubuntu.com/download/desktop)
* Create bootable USB, install OR use WSL 2 on Windows / Docker on Mac.

**Verify installation:**
```bash
lsb_release -a
# Output should show: Release: 20.04
````

-----

## Step 2: Install ROS Noetic

**Full ROS Desktop Installation:**

```bash
# Setup sources list
sudo sh -c 'echo "deb [http://packages.ros.org/ros/ubuntu](http://packages.ros.org/ros/ubuntu) $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add keys
curl -s [https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc](https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc) | sudo apt-key add -

# Update and install
sudo apt-get update
sudo apt-get install -y ros-noetic-desktop-full

# Source setup script
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install build dependencies
sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

**Verify ROS installation:**

```bash
roscore
# Should start without errors. Press Ctrl+C to stop.
```

-----

## Step 3: Install Gazebo 11

```bash
# Install Gazebo
sudo apt-get install -y gazebo11 libgazebo11-dev

# Install ROS-Gazebo bridge
sudo apt-get install -y ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Verify Gazebo
gazebo --version
# Output should show: Gazebo multi-robot simulator, version 11.x.x
```

-----

## Step 4: Install PX4 Autopilot (Simulator)

```bash
# Clone PX4-Autopilot repository
git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) ~/PX4-Autopilot --recursive
cd ~/PX4-Autopilot

# Run setup script (Ubuntu 20.04)
bash ./Tools/setup/ubuntu.sh

# Verify installation
make px4_sitl gazebo
# Should launch Gazebo with simulated drone
# Press Ctrl+C to exit
```

-----

## Step 5: Install Dependencies

```bash
# Install build essentials
sudo apt-get install -y build-essential cmake git wget

# Install point cloud library (PCL)
sudo apt-get install -y libpcl-dev

# Install Eigen3
sudo apt-get install -y libeigen3-dev

# Install MAVSDK
sudo apt-get install -y ros-noetic-mavsdk ros-noetic-mavsdk-msgs

# Install additional ROS packages
sudo apt-get install -y ros-noetic-tf2 ros-noetic-sensor-msgs ros-noetic-geometry-msgs
```

**Verify installations:**

```bash
pkg-config --modversion pcl_common
# Output: 1.x.x

cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep "EIGEN_WORLD_VERSION"
# Output should show version
```

-----

## Step 6: Clone Drone-Simulation Repository

```bash
# Clone repository
git clone [https://github.com/ananya-asa/Drone-Simulation.git](https://github.com/ananya-asa/Drone-Simulation.git) ~/Drone-Simulation
cd ~/Drone-Simulation

# Verify structure
ls -la
# Should show: CMakeLists.txt, src/, include/, worlds/, docs/
```

-----

## Step 7: Build the Project

```bash
cd ~/Drone-Simulation

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build (using 4 parallel jobs)
make -j4

# Verify build
ls -la
# Should show: uav_planner (executable)
```

**If build fails:**

```bash
# Clean and rebuild
make clean
cmake ..
make -j4

# OR with verbose output for debugging
make VERBOSE=1
```

-----

## Step 8: Configure PX4 + Gazebo

**Create/verify Gazebo world file:**

```bash
# Copy default world if it doesn't exist
cp ~/Drone-Simulation/worlds/default.sdf ~/.gazebo/models/

# Verify Gazebo can find models
gazebo --verbose ~/.gazebo/models/
```

-----

## Step 9: Run the System (Full Setup)

**Terminal 1: Start PX4 SITL + Gazebo**

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
# Keep this running, you'll see drone in Gazebo window
```

**Terminal 2: Source ROS and launch LIDAR simulator**

```bash
source ~/.bashrc
roslaunch gazebo_ros spawn_model_state_publisher.launch
# This publishes simulated LIDAR data to /lidar topic
```

**Terminal 3: Run Drone Planner**

```bash
cd ~/Drone-Simulation/build
./uav_planner
# You'll be prompted to enter goal coordinates
```

**Terminal 4 (Optional): Monitor ROS Topics**

```bash
source ~/.bashrc
rostopic list     # See all topics
rostopic echo /lidar   # Echo LIDAR data
```

-----

## Step 10: Run First Test Mission

In **Terminal 3 (uav\_planner)**:

```text
[INPUT] Enter goal position in NED coordinates:
  North (meters): 2
  East (meters): 0
  Down (meters, put -2.5 for 2.5m up): -2.5
```

**Expected output:**

```text
[FLIGHT] Arming drone...
[FLIGHT] Taking off...
[FLIGHT] Starting Offboard mode...
[MISSION] Starting dynamic replanning mission...
[LIDAR] Data received! Building map with 6185 points
[PLANNER] ✓ Path found in 58 iterations! (53 nodes expanded)
[ACT] Flying next 3 waypoints...
Waypoint 1/3: ... ✓ Reached
Waypoint 2/3: ... ✓ Reached
Waypoint 3/3: ... ✓ Reached
[GOAL REACHED]
[FLIGHT] Mission complete. Landing...
[Landing detected]
--- MISSION COMPLETE ---
```

**In Gazebo (Terminal 1):**

1.  Drone takes off vertically
2.  Drone moves toward goal
3.  Drone lands

-----

## Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **"PX4 not found"** | Make sure PX4-Autopilot is built: <br> `cd ~/PX4-Autopilot && make clean && make px4_sitl gazebo` |
| **"LIDAR topic not published"** | Check ROS topics: `rostopic list | grep lidar`. <br> Ensure Gazebo LIDAR plugin is loaded in `.sdf` file. |
| **"CMake configuration fails"** | Clear cache: `cd build && rm -rf * && cmake .. && make -j4` |
| **"MAVSDK connection refused"** | Ensure PX4 is running on `localhost:14540`. <br> PX4 terminal should show: `[mavsdk_server] Server started`. |
| **"Permission denied"** | `chmod +x ~/Drone-Simulation/build/uav_planner` |

-----

## Parameter Tuning

### Adjust Safety Margin

Edit `src/world_model.cpp`:

```cpp
const float SAFETY_MARGIN = 0.4f;  // Default: 0.4m

// Increase for safer flights:
// const float SAFETY_MARGIN = 0.7f;

// Decrease for tighter passages:
// const float SAFETY_MARGIN = 0.2f;
```

*Then rebuild:* `make -j4`

### Adjust Grid Resolution

Edit `src/world_model.cpp`:

```cpp
const double RESOLUTION = 0.1;  // Default: 0.1m (10cm)

// Finer grid (slower planning, better accuracy):
// const double RESOLUTION = 0.05;

// Coarser grid (faster planning):
// const double RESOLUTION = 0.2;
```

### Adjust Waypoint Tolerance

Edit `src/trajectory_follower.cpp`:

```cpp
const float WAYPOINT_TOLERANCE = 0.3f;  // Default: 0.3m

// Stricter tolerance (fly closer to waypoints):
// const float WAYPOINT_TOLERANCE = 0.1f;

// Looser tolerance (faster waypoint completion):
// const float WAYPOINT_TOLERANCE = 0.5f;
```

-----

## Advanced Setup

### Use GPU Acceleration (CUDA)

If you have an NVIDIA GPU:

```bash
# Install CUDA Toolkit
# Download from: [https://developer.nvidia.com/cuda-downloads](https://developer.nvidia.com/cuda-downloads)

# After installation, rebuild with CUDA support
cd ~/Drone-Simulation/build
cmake .. -DCUDA_ENABLED=ON
make -j4
```

### Run with Custom World

1.  Create your own world file (`.sdf` format)
2.  Place in `worlds/` directory
3.  Specify in PX4 startup:

<!-- end list -->

```bash
make px4_sitl gazebo_custom_world
```

### Enable Debug Logging

Edit `src/main.cpp`:

```cpp
#define DEBUG_MODE 1
#define LOG_LEVEL DEBUG  // Options: DEBUG, INFO, WARNING, ERROR
```

Rebuild and run to see verbose output.

-----

## Uninstall (If Needed)

```bash
# Remove ROS
sudo apt-get remove -y ros-noetic-*
sudo apt-get autoremove -y

# Remove Gazebo
sudo apt-get remove -y gazebo11 libgazebo11-dev

# Remove cloned repositories
rm -rf ~/PX4-Autopilot
rm -rf ~/Drone-Simulation
```

-----

## Performance Verification

**Check CPU/Memory Usage:**

```bash
# Terminal 1: Monitor system resources
top -b -n 1 | grep -E "Cpu|Mem"

# Terminal 2: Specific process monitoring
ps aux | grep uav_planner
```

**Expected on Intel i5 laptop:**

  * CPU: 40-60%
  * Memory: 150-200MB
  * Frame rate: 5Hz planning, 20Hz control

-----

## Next Steps After Successful Setup

  * ✅ Test different goal coordinates
  * ✅ Add obstacles in Gazebo manually
  * ✅ Read through `ARCHITECTURE.md`
  * ✅ Explore parameter tuning
  * ✅ Prepare for Phase 2 (Kalman Filter implementation)

-----

## Getting Help

**For issues:**

1.  Check this troubleshooting section
2.  Check console output for error messages
3.  Open an issue on GitHub with:
      * OS version
      * Error message (full output)
      * Steps to reproduce

**Recommended resources:**

  * [ROS Documentation](http://wiki.ros.org/)
  * [Gazebo Documentation](http://gazebosim.org/tutorials)
  * [PX4 Documentation](https://docs.px4.io/)

**Last Updated:** November 27, 2025
**Setup Version:** Ubuntu 20.04 + ROS Noetic
**Status:** Tested & Verified

```
```
