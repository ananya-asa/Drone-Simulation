// FILE: src/main.cpp
// VERSION: 6.0 (IMPROVED)
// IMPROVEMENTS: Memory management, error handling, 3D support, documentation
// ============================================================================

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <future>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

// MAVSDK Includes
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Project Headers
#include "path_planner.h"
#include "world_model.h"
#include "trajectory_follower.h"
#include "lidar_bridge.h"
#include <Eigen/Dense>
#include "kalman_filter.h"

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================
// Grid and Physics
constexpr double RESOLUTION = 0.1;              // 10cm grid resolution
constexpr double GOAL_TOLERANCE = 0.5;          // 0.5m goal tolerance
constexpr float SAFETY_MARGIN = 0.8f;           // 0.8m collision buffer
constexpr float WAYPOINT_TOLERANCE = 0.3f;      // 0.3m waypoint tolerance

// Mission Parameters
constexpr int MAX_CYCLES = 50;                  // Maximum replanning cycles
constexpr int MAX_FAILS = 5;                    // Max consecutive planning failures
constexpr int CONTROL_RATE_HZ = 20;             // 20Hz control frequency
constexpr int COMMAND_PERIOD_MS = 50;           // 1000/20Hz = 50ms

// Flight Parameters
constexpr float TAKEOFF_ALTITUDE = 2.5f;        // 2.5m altitude (NED: -2.5)
constexpr float MIN_FLIGHT_ALT = 0.5f;          // Minimum flight altitude
constexpr float MAX_FLIGHT_ALT = 20.0f;         // Maximum flight altitude
constexpr float SAFE_ALTITUDE = 1.5f;           // Safe altitude for planning start

// ============================================================================
// FLIGHT LOGGER CLASS (Data Collection & Analysis)
// ============================================================================
class FlightLogger {
private:
    std::ofstream log_file;
    int log_count = 0;

public:
    // Constructor: Opens CSV file with headers
    FlightLogger(const std::string& filename) {
        log_file.open(filename);
        if (log_file.is_open()) {
            // CSV Header Row
            log_file << "Timestamp,Cycle,North_m,East_m,Down_m,GridX,GridY,GridZ,"
                     << "GoalGridX,GoalGridY,GoalGridZ,DistanceToGoal_m,PathLength,"
                     << "LidarPoints,PlanningStatus,WaypointsFlown,KalmanDriftm\n";
            std::cout << "[LOG] 📊 Flight data will be saved to: " << filename << std::endl;
        } else {
            std::cerr << "[ERROR] ❌ Could not open log file: " << filename << std::endl;
        }
    }
    
    // Destructor: Closes and flushes log
    ~FlightLogger() {
        if (log_file.is_open()) {
            log_file.close();
            std::cout << "[LOG] ✅ Flight data saved (" << log_count << " cycles logged)." << std::endl;
        }
    }
    
    // Log a single mission cycle with comprehensive telemetry
    void log_cycle(
        int cycle,
        const mavsdk::Telemetry::PositionNed& pos,
        const Eigen::Vector3i& start_grid,
        const Eigen::Vector3i& goal_grid,
        float distance_to_goal,
        int path_length,
        int lidar_points,
        const std::string& status,
        int waypoints_flown,
        float kalman_drift = 0.0f) {
        
        if (!log_file.is_open()) return;
        
        // Format timestamp
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        
        // Write CSV row
        log_file << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << ","
                 << cycle << ","
                 << pos.north_m << "," << pos.east_m << "," << pos.down_m << ","
                 << start_grid.x() << "," << start_grid.y() << "," << start_grid.z() << ","
                 << goal_grid.x() << "," << goal_grid.y() << "," << goal_grid.z() << ","
                 << distance_to_goal << "," << path_length << ","
                 << lidar_points << "," << status << "," << waypoints_flown << ","
                 << kalman_drift << "\n";
        
        log_file.flush();
        log_count++;
    }
};

// ============================================================================
// MAVSDK SYSTEM DISCOVERY
// ============================================================================
std::shared_ptr<System> get_system(Mavsdk& mavsdk) {
    std::cout << "⏳ Waiting to discover drone..." << std::endl;
    auto prom = std::promise<std::shared_ptr<System>>();
    auto fut = prom.get_future();

    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().at(0);
        if (system->has_autopilot()) {
            std::cout << "✅ Discovered autopilot!" << std::endl;
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    // 10-second timeout for drone discovery
    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cerr << "❌ No drone found after 10 seconds. Check connection." << std::endl;
        return nullptr;
    }
    return fut.get();
}

// ============================================================================
// MAIN MISSION PROGRAM
// ============================================================================
int main() {
    std::cout << "\n🚁 ======================================" << std::endl;
    std::cout << "    Autonomous Drone Planner V6.0" << std::endl;
    std::cout << "    Enhanced with Smart Memory Management" << std::endl;
    std::cout << "    3D Path Planning + Kalman Filtering" << std::endl;
    std::cout << "======================================\n" << std::endl;

    // ========================================================================
    // INITIALIZATION PHASE
    // ========================================================================
    
    // World Model Setup
    std::cout << "[INIT] ⚙️  Creating world model (resolution: " << RESOLUTION << "m)..." << std::endl;
    WorldModel world(RESOLUTION);
    
    // Kalman Filter Setup (6-state: position + velocity)
    std::cout << "[INIT] 📊 Initializing Kalman Filter (6-DOF state)..." << std::endl;
    KalmanFilter kalman_filter;
    std::cout << "[INIT] ✅ Kalman Filter ready" << std::endl;

    // Flight Logger Setup
    FlightLogger logger("flight_log.csv");
    
    // LIDAR Bridge Setup
    std::cout << "[INIT] 📡 Initializing LIDAR sensor bridge..." << std::endl;
    LIDARBridge lidar;

    // ========================================================================
    // MAVSDK SETUP
    // ========================================================================
    std::cout << "[CONNECT] 🌐 Connecting to MAVSDK..." << std::endl;
    Mavsdk mavsdk(Mavsdk::Configuration(mavsdk::ComponentType::GroundStation));
    
    ConnectionResult connection_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "[ERROR] ❌ MAVSDK connection failed: " << connection_result << std::endl;
        return 1;
    }
    std::cout << "[CONNECT] ✅ MAVSDK listening on UDP 14540" << std::endl;
    
    // Discover drone system
    auto system = get_system(mavsdk);
    if (!system) {
        std::cerr << "[ERROR] ❌ Failed to discover drone system" << std::endl;
        return 1;
    }
    
    // Get plugin pointers
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);
    
    std::cout << "[CONNECT] ✅ All plugins initialized" << std::endl;

    // ========================================================================
    // USER INPUT: GOAL COORDINATES
    // ========================================================================
    std::cout << "\n📍 [INPUT] Enter goal position (NED coordinates):" << std::endl;
    
    float goal_north, goal_east, goal_down;
    
    std::cout << "  North (meters): ";
    std::cin >> goal_north;
    
    std::cout << "  East (meters): ";
    std::cin >> goal_east;
    
    std::cout << "  Down (meters, use -2.5 for 2.5m altitude up): ";
    std::cin >> goal_down;
    
    // Convert goal to grid coordinates
    Eigen::Vector3d goal_ned(goal_north, goal_east, goal_down);
    Eigen::Vector3i goal_grid(
        std::round(goal_north / RESOLUTION),
        std::round(goal_east / RESOLUTION),
        std::round(goal_down / RESOLUTION)
    );
    
    std::cout << "\n[DEBUG] ✅ Goal NED: " << goal_ned.transpose() << std::endl;
    std::cout << "[DEBUG] ✅ Goal Grid: " << goal_grid.transpose() << std::endl;

    // ========================================================================
    // LIDAR SUBSCRIPTION & WARM-UP
    // ========================================================================
    std::cout << "\n[SETUP] 📡 Subscribing to LIDAR sensor..." << std::endl;
    if (!lidar.subscribe()) {
        std::cerr << "[ERROR] ❌ LIDAR subscription failed" << std::endl;
        return 1;
    }
    std::cout << "[SETUP] ✅ LIDAR subscribed. Warming up..." << std::endl;
    sleep_for(seconds(2));  // Allow sensor to stabilize

    // ========================================================================
    // PRE-FLIGHT CHECKS
    // ========================================================================
    std::cout << "\n[PREFLIGHT] 🔍 Running pre-flight health checks..." << std::endl;
    while (!telemetry->health_all_ok()) {
        std::cout << "[PREFLIGHT] ⏳ Waiting for health checks..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "[PREFLIGHT] ✅ All systems healthy" << std::endl;

    // ========================================================================
    // ARM DRONE
    // ========================================================================
    std::cout << "\n[FLIGHT] 🔒 Arming drone..." << std::endl;
    Action::Result arm_result = action->arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "[ERROR] ❌ Arming failed: " << arm_result << std::endl;
        return 1;
    }
    std::cout << "[FLIGHT] ✅ Drone armed" << std::endl;

    // ========================================================================
    // TAKEOFF
    // ========================================================================
    std::cout << "\n[FLIGHT] ⬆️  Taking off to " << TAKEOFF_ALTITUDE << "m..." << std::endl;
    Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "[ERROR] ❌ Takeoff failed: " << takeoff_result << std::endl;
        return 1;
    }

    // Wait for safe altitude before planning
    std::cout << "[FLIGHT] 📏 Waiting for safe altitude (" << SAFE_ALTITUDE << "m)..." << std::endl;
    int altitude_check_count = 0;
    while (altitude_check_count < 100) {
        float current_altitude = -telemetry->position_velocity_ned().position.down_m;
        if (current_altitude >= SAFE_ALTITUDE) {
            std::cout << "[FLIGHT] ✅ Altitude reached: " << current_altitude << "m" << std::endl;
            break;
        }
        sleep_for(milliseconds(100));
        altitude_check_count++;
    }

    // ========================================================================
    // ENTER OFFBOARD MODE
    // ========================================================================
    std::cout << "\n[FLIGHT] 🤖 Entering OFFBOARD mode..." << std::endl;
    
    // Set hold position
    Offboard::PositionNedYaw hold_position{};
    hold_position.north_m = 0.0f;
    hold_position.east_m = 0.0f;
    hold_position.down_m = -TAKEOFF_ALTITUDE;
    hold_position.yaw_deg = 0.0f;
    offboard->set_position_ned(hold_position);
    
    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "[ERROR] ❌ Offboard mode failed: " << offboard_result << std::endl;
        return 1;
    }
    std::cout << "[FLIGHT] ✅ OFFBOARD mode active" << std::endl;
    sleep_for(seconds(1));

    // ========================================================================
    // INITIALIZE KALMAN FILTER WITH CURRENT POSITION
    // ========================================================================
    std::cout << "\n[KALMAN] 🔧 Initializing Kalman Filter..." << std::endl;
    Telemetry::PositionNed initial_pos = telemetry->position_velocity_ned().position;
    Eigen::Vector3f init_pos(initial_pos.north_m, initial_pos.east_m, initial_pos.down_m);
    
    // Initialize with current position, dt=50ms (20Hz)
    kalman_filter.init(init_pos, 1.0f / CONTROL_RATE_HZ);
    std::cout << "[KALMAN] ✅ Filter initialized at: " << init_pos.transpose() << std::endl;

    // ========================================================================
    // MAIN MISSION LOOP: DYNAMIC REPLANNING
    // ========================================================================
    std::cout << "\n🚀 [MISSION] Starting autonomous navigation with dynamic replanning..." << std::endl;
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

    int fail_count = 0;
    int total_waypoints_flown = 0;
    float cumulative_kalman_drift = 0.0f;

    for (int cycle = 0; cycle < MAX_CYCLES; ++cycle) {
        
        // Safety check: Is drone still in air?
        if (!telemetry->in_air()) {
            std::cout << "\n⚠️  [MISSION] Drone is not in air. Aborting mission." << std::endl;
            break;
        }

        // ====================================================================
        // STEP 1: LIDAR DATA ACQUISITION
        // ====================================================================
        int wait_count = 0;
        const int max_wait = 30;  // 3 seconds max
        
        while (!lidar.has_new_data() && wait_count < max_wait) {
            sleep_for(milliseconds(100));
            wait_count++;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (lidar.has_new_data()) {
            cloud = lidar.get_cloud();
            lidar.reset_data_flag();
            
            // Validation: Check cloud quality
            if (cloud->points.size() < 100) {
                std::cout << "[LIDAR] ⚠️  Warning: Low point count (" << cloud->points.size() << ")" << std::endl;
            }
        } else {
            // Fallback: Generate synthetic cloud (ONLY if no real data)
            std::cout << "[LIDAR] ⚠️  Using fallback point cloud (sensor timeout)" << std::endl;
            cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for (double y = -1.0; y <= 1.0; y += 0.1)
                for (double z = -1.0; z <= 1.0; z += 0.1)
                    cloud->points.push_back(pcl::PointXYZ(0.5, y, z));
        }
        
        // Build octree from point cloud
        world.buildMap(cloud);

        // ====================================================================
        // STEP 2: KALMAN FILTER UPDATE (Sensor Fusion)
        // ====================================================================
        Telemetry::PositionNed raw_pos = telemetry->position_velocity_ned().position;
        Eigen::Vector3f raw_position(raw_pos.north_m, raw_pos.east_m, raw_pos.down_m);
        
        // Predict: Use motion model
        kalman_filter.predict();
        
        // Update: Fuse LIDAR/telemetry measurement
        kalman_filter.update(raw_position);
        
        // Get filtered (smoothed) position
        Eigen::Vector3f filtered_position = kalman_filter.getPosition();
        Eigen::Vector3f estimated_velocity = kalman_filter.getVelocity();
        
        // Calculate Kalman drift (difference from raw measurement)
        float kalman_drift = (raw_position - filtered_position).norm();
        cumulative_kalman_drift += kalman_drift;
        
        std::cout << "\n[CYCLE] 🔄 Cycle " << cycle + 1 << "/" << MAX_CYCLES << std::endl;
        std::cout << "  Raw Position:      " << raw_position.transpose() << " m" << std::endl;
        std::cout << "  Filtered Position: " << filtered_position.transpose() << " m" << std::endl;
        std::cout << "  Estimated Velocity: " << estimated_velocity.transpose() << " m/s" << std::endl;
        std::cout << "  Kalman Drift: " << kalman_drift << " m" << std::endl;

        // ====================================================================
        // STEP 3: CONVERT TO GRID COORDINATES
        // ====================================================================
        Eigen::Vector3d start_ned(filtered_position.x(), filtered_position.y(), filtered_position.z());
        
        Eigen::Vector3i start_grid(
            std::round(filtered_position.x() / RESOLUTION),
            std::round(filtered_position.y() / RESOLUTION),
            std::round(filtered_position.z() / RESOLUTION)
        );
        
        std::cout << "  Grid Position: (" << start_grid.x() << ", " 
                  << start_grid.y() << ", " << start_grid.z() << ")" << std::endl;

        // ====================================================================
        // STEP 4: PATH PLANNING (A* Algorithm)
        // ====================================================================
        auto start_time = std::chrono::high_resolution_clock::now();
        std::vector<Eigen::Vector3i> path_grid = A_star_search(start_grid, goal_grid, world);
        auto end_time = std::chrono::high_resolution_clock::now();
        float planning_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();

        if (path_grid.empty()) {
            std::cerr << "  ❌ [PLAN] No path found. Hovering..." << std::endl;
            logger.log_cycle(cycle + 1, raw_pos, start_grid, goal_grid, 
                           -1.0f, 0, cloud->points.size(), "NO_PATH", 0, kalman_drift);
            fail_count++;
            
            if (fail_count >= MAX_FAILS) {
                std::cerr << "  💥 [PLAN] Maximum failures exceeded. Landing." << std::endl;
                break;
            }
            sleep_for(seconds(1));
            continue;
        }
        
        fail_count = 0;  // Reset failure counter on success
        std::cout << "  ✅ [PLAN] Path found in " << planning_time_ms << "ms" << std::endl;
        std::cout << "  📍 [PLAN] " << path_grid.size() << " waypoints" << std::endl;

        // ====================================================================
        // STEP 5: TRAJECTORY FOLLOWING (Fly next segment)
        // ====================================================================
        // Only fly first 3 waypoints, then replan (reactive planning)
        int seg_end = std::min(3, static_cast<int>(path_grid.size()));
        std::vector<Eigen::Vector3i> this_leg(path_grid.begin(), path_grid.begin() + seg_end);
        
        std::cout << "  ✈️  [ACT] Flying segment: " << seg_end << " waypoints" << std::endl;
        
        TrajectoryFollower follower(telemetry, offboard, this_leg, RESOLUTION);
        follower.start();
        
        total_waypoints_flown += seg_end;

        // ====================================================================
        // STEP 6: DISTANCE CHECK & GOAL TEST
        // ====================================================================
        Telemetry::PositionNed current_pos = telemetry->position_velocity_ned().position;
        
        float distance_north = current_pos.north_m - goal_ned.x();
        float distance_east = current_pos.east_m - goal_ned.y();
        float distance_down = current_pos.down_m - goal_ned.z();
        float distance_to_goal = std::sqrt(distance_north*distance_north + 
                                          distance_east*distance_east + 
                                          distance_down*distance_down);

        std::cout << "  📍 [STATUS] Distance to goal: " << distance_to_goal << "m" << std::endl;
        
        logger.log_cycle(cycle + 1, current_pos, start_grid, goal_grid, distance_to_goal,
                        path_grid.size(), cloud->points.size(), "SUCCESS", seg_end, kalman_drift);

        // Check if goal reached
        if (distance_to_goal < GOAL_TOLERANCE) {
            std::cout << "\n🎉 [SUCCESS] GOAL REACHED!" << std::endl;
            std::cout << "  Distance: " << distance_to_goal << "m (tolerance: " << GOAL_TOLERANCE << "m)" << std::endl;
            break;
        }
    }

    // ========================================================================
    // LANDING & CLEANUP
    // ========================================================================
    std::cout << "\n🛬 [FLIGHT] Mission complete. Exiting OFFBOARD mode..." << std::endl;
    offboard->stop();
    sleep_for(seconds(2));
    
    std::cout << "[FLIGHT] 🛬 Landing drone..." << std::endl;
    action->land();
    
    // Wait for landing
    int landing_timeout = 0;
    while (telemetry->in_air() && landing_timeout < 60) {
        sleep_for(seconds(1));
        landing_timeout++;
    }
    
    if (telemetry->in_air()) {
        std::cerr << "⚠️  [FLIGHT] Landing timeout - drone may still be in air" << std::endl;
    } else {
        std::cout << "[FLIGHT] ✅ Drone landed safely" << std::endl;
    }

    // ========================================================================
    // MISSION SUMMARY
    // ========================================================================
    std::cout << "\n📊 ═══════════════════════════════════════" << std::endl;
    std::cout << "    MISSION COMPLETE" << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;
    std::cout << "  Total Waypoints Flown: " << total_waypoints_flown << std::endl;
    std::cout << "  Avg Kalman Drift: " << (cumulative_kalman_drift / MAX_CYCLES) << "m" << std::endl;
    std::cout << "  Flight Log: flight_log.csv" << std::endl;
    std::cout << "═══════════════════════════════════════\n" << std::endl;

    return 0;
}