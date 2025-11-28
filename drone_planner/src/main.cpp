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

// MAVSDK Includes
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Our Project Headers
#include "path_planner.h"
#include "world_model.h"
#include "trajectory_follower.h"
#include "lidar_bridge.h"
#include <Eigen/Dense>
#include "kalman_filter.h"  // ✅ Kalman Filter

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

// ============================================================================
// FLIGHT LOGGER CLASS (For Data Collection)
// ============================================================================
class FlightLogger {
private:
    std::ofstream log_file;
public:
    FlightLogger(const std::string& filename) {
        log_file.open(filename);
        if (log_file.is_open()) {
            log_file << "Timestamp,Cycle,North_m,East_m,Down_m,GridX,GridY,GridZ,"
                     << "GoalGridX,GoalGridY,GoalGridZ,DistanceToGoal_m,PathLength,"
                     << "LidarPoints,PlanningStatus,WaypointsFlown\n";
            std::cout << "[LOG] 📊 Flight data will be saved to: " << filename << std::endl;
        }
    }
    
    ~FlightLogger() {
        if (log_file.is_open()) {
            log_file.close();
            std::cout << "[LOG] ✅ Flight data saved successfully." << std::endl;
        }
    }
    
    void log_cycle(int cycle, 
                   const mavsdk::Telemetry::PositionNed& pos,
                   const Eigen::Vector3i& start_grid,
                   const Eigen::Vector3i& goal_grid,
                   float distance_to_goal,
                   int path_length,
                   int lidar_points,
                   const std::string& status,
                   int waypoints_flown) {
        if (!log_file.is_open()) return;
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        log_file << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << ","
                 << cycle << ","
                 << pos.north_m << "," << pos.east_m << "," << pos.down_m << ","
                 << start_grid.x() << "," << start_grid.y() << "," << start_grid.z() << ","
                 << goal_grid.x() << "," << goal_grid.y() << "," << goal_grid.z() << ","
                 << distance_to_goal << "," << path_length << ","
                 << lidar_points << "," << status << "," << waypoints_flown << "\n";
        log_file.flush();
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

    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cerr << "❌ No drone found, timeout!" << std::endl;
        return nullptr;
    }
    return fut.get();
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================
int main() {
    std::cout << "\n🚁 === Autonomous Drone Planner V5.5 (with Kalman Filter) ===" << std::endl;

    const double RESOLUTION = 0.1; // 10cm grid resolution
    WorldModel world(RESOLUTION);
    
    // 🆕 ====== KALMAN FILTER CREATION ======
    KalmanFilter kalman_filter;  // 6-state Kalman (position + velocity)
    std::cout << "[INIT] ✅ Kalman Filter created" << std::endl;

    FlightLogger logger("flight_log.csv");
    LIDARBridge lidar;

    // -------- MAVSDK Setup --------
    Mavsdk mavsdk(Mavsdk::Configuration(mavsdk::ComponentType::GroundStation));
    
    ConnectionResult connection_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "[ERROR] ❌ Connection failed: " << connection_result << std::endl;
        return 1;
    }
    
    auto system = get_system(mavsdk);
    if (!system) return 1;
    
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    // -------- User Input for Goal --------
    std::cout << "\n📍 [INPUT] Enter goal position in NED coordinates:" << std::endl;
    std::cout << "  North (meters): "; 
    float goal_north; 
    std::cin >> goal_north;
    
    std::cout << "  East (meters): "; 
    float goal_east; 
    std::cin >> goal_east;
    
    std::cout << "  Down (meters, put -2.5 for 2.5m up): "; 
    float goal_down; 
    std::cin >> goal_down;
    
    // Convert Goal to Grid Manually (Safe & Reliable)
    Eigen::Vector3d goal_ned(goal_north, goal_east, goal_down);
    Eigen::Vector3i goal_grid(
        std::round(goal_north / RESOLUTION),
        std::round(goal_east / RESOLUTION),
        std::round(goal_down / RESOLUTION)
    );
    
    std::cout << "[DEBUG] 📋 Goal NED: " << goal_ned.transpose() << std::endl;
    std::cout << "[DEBUG] 📋 Goal Grid: " << goal_grid.transpose() << std::endl;

    // -------- LIDAR Setup --------
    std::cout << "\n📡 [SETUP] Initializing LIDAR sensor bridge..." << std::endl;
    if (!lidar.subscribe()) {
        std::cerr << "[ERROR] ❌ Failed to subscribe to LIDAR" << std::endl;
        return 1;
    }
    // Give sensor a moment to warm up
    sleep_for(seconds(2));

    // -------- Pre-flight Checks --------
    std::cout << "\n🔍 [SETUP] Waiting for drone to be ready..." << std::endl;
    while (!telemetry->health_all_ok()) {
        sleep_for(seconds(1));
    }

    // -------- Arming --------
    std::cout << "\n🔒 [FLIGHT] Arming drone..." << std::endl;
    if (action->arm() != Action::Result::Success) {
        std::cerr << "[ERROR] ❌ Arming failed" << std::endl;
        return 1;
    }

    // -------- Takeoff --------
    std::cout << "⬆️  [FLIGHT] Taking off..." << std::endl;
    if (action->takeoff() != Action::Result::Success) {
        std::cerr << "[ERROR] ❌ Takeoff failed" << std::endl;
        return 1;
    }

    // --- Wait for valid altitude before planning ---
    std::cout << "📏 [FLIGHT] Waiting for safe altitude (1.5m)..." << std::endl;
    while (true) {
        float altitude = -1.0f * telemetry->position_velocity_ned().position.down_m;
        if (altitude >= 1.5f) {
            std::cout << "✅ [FLIGHT] Altitude reached: " << altitude << "m" << std::endl;
            break;
        }
        sleep_for(milliseconds(200));
    }

    // -------- Offboard Mode --------
    std::cout << "\n🤖 [FLIGHT] Starting Offboard mode..." << std::endl;
    Offboard::PositionNedYaw stay{};
    stay.north_m = 0.0f; 
    stay.east_m = 0.0f; 
    stay.down_m = -2.5f; 
    stay.yaw_deg = 0.0f;
    offboard->set_position_ned(stay);
    
    if (offboard->start() != Offboard::Result::Success) {
        std::cerr << "[ERROR] ❌ Offboard start failed" << std::endl;
        return 1;
    }
    sleep_for(seconds(1));

    // 🆕 ====== KALMAN FILTER INITIALIZATION ======
    std::cout << "\n⚙️  [KALMAN] Initializing Kalman Filter..." << std::endl;
    Telemetry::PositionNed initial_pos = telemetry->position_velocity_ned().position;
    Eigen::Vector3f init_pos(initial_pos.north_m, initial_pos.east_m, initial_pos.down_m);
    kalman_filter.init(init_pos, 0.05f);  // 0.05s = 20Hz control rate
    std::cout << "[KALMAN] ✅ Filter initialized with position: " << init_pos.transpose() << std::endl;

    // ====================================================================
    // MISSION LOOP
    // ====================================================================
    std::cout << "\n🚀 [MISSION] Starting dynamic replanning mission..." << std::endl;

    const double GOAL_TOLERANCE = 0.5;
    const int MAX_CYCLES = 50;
    const int MAX_FAILS = 5;
    int fail_count = 0;

    for (int cycle = 0; cycle < MAX_CYCLES; ++cycle) {
        
        if (!telemetry->in_air()) {
            std::cout << "\n⚠️  [MISSION] Drone is not in air. Aborting mission." << std::endl;
            break;
        }

        // -------- LIDAR Acquisition --------
        int wait_count = 0;
        while (!lidar.has_new_data() && wait_count < 30) {
            sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (lidar.has_new_data()) {
            cloud = lidar.get_cloud();
            lidar.reset_data_flag();
        } else {
            // Fallback Cloud (Safety)
            cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for (double y = -1.0; y <= 1.0; y += 0.1)
                for (double z = -1.0; z <= 1.0; z += 0.1)
                    cloud->points.push_back(pcl::PointXYZ(0.5, y, z));
        }
        
        world.buildMap(cloud);

        // 🆕 ====== KALMAN FILTER UPDATE ======
        Telemetry::PositionNed pos = telemetry->position_velocity_ned().position;
        Eigen::Vector3f raw_pos(pos.north_m, pos.east_m, pos.down_m);
        
        // Predict next state
        kalman_filter.predict();
        
        // Update with new measurement
        kalman_filter.update(raw_pos);
        
        // Get smoothed position
        Eigen::Vector3f filtered_pos = kalman_filter.getPosition();
        
        // Optional: Log raw vs filtered for debugging
        std::cout << "[KALMAN] 📊 Raw: " << raw_pos.transpose() 
                  << " | Filtered: " << filtered_pos.transpose() << std::endl;

        // Use filtered position for planning
        Eigen::Vector3d start_ned(filtered_pos.x(), filtered_pos.y(), filtered_pos.z());
        
        Eigen::Vector3i start_grid(
            std::round(filtered_pos.x() / RESOLUTION),
            std::round(filtered_pos.y() / RESOLUTION),
            std::round(filtered_pos.z() / RESOLUTION)
        );

        std::cout << "[CYCLE] 🔄 " << cycle << " | Filtered Pos: " << start_ned.transpose() << std::endl;

        // -------- Path Planning (A*) --------
        auto path_grid = A_star_search(start_grid, goal_grid, world);

        if (path_grid.empty()) {
            std::cerr << "[PLAN] ❌ No path found (Blocked/Collision). Hovering..." << std::endl;
            logger.log_cycle(cycle + 1, pos, start_grid, goal_grid, -1.0f, 0, cloud->points.size(), "NO_PATH", 0);
            fail_count++;
            
            if (fail_count >= MAX_FAILS) {
                std::cerr << "[PLAN] 💥 Maximum replanning failures exceeded. Landing." << std::endl;
                break;
            }
            sleep_for(seconds(1));
            continue;
        }
        
        fail_count = 0;

        // -------- Trajectory Following --------
        std::cout << "[PLAN] ✅ Path found: " << path_grid.size() << " waypoints" << std::endl;
        
        // Fly a small segment of the path (e.g. 3 steps) then re-plan
        int seg_end = std::min(3, static_cast<int>(path_grid.size()));
        std::vector<Eigen::Vector3i> this_leg(path_grid.begin(), path_grid.begin() + seg_end);
        
        std::cout << "[ACT] ✈️  Flying next " << seg_end << " waypoints..." << std::endl;
        
        TrajectoryFollower follower(telemetry, offboard, this_leg, RESOLUTION);
        follower.start();

        // -------- Distance Check --------
        Telemetry::PositionNed curr = telemetry->position_velocity_ned().position;
        
        float d_n = curr.north_m - goal_ned.x();
        float d_e = curr.east_m - goal_ned.y();
        float d_d = curr.down_m - goal_ned.z();
        float distance_to_goal = std::sqrt(d_n*d_n + d_e*d_e + d_d*d_d);

        std::cout << "[STATUS] 📍 Real Distance to goal: " << distance_to_goal << "m" << std::endl;
        
        logger.log_cycle(cycle + 1, curr, start_grid, goal_grid, distance_to_goal,
                    path_grid.size(), cloud->points.size(), "SUCCESS", seg_end);

        // Check if goal reached
        if (distance_to_goal < GOAL_TOLERANCE) {
            std::cout << "\n🎉 [GOAL REACHED] Distance " << distance_to_goal << "m is within tolerance " << GOAL_TOLERANCE << "m\n" << std::endl;
            break;
        }
    }

    // ====================================================================
    // LANDING & CLEANUP
    // ====================================================================
    std::cout << "\n🛬 [FLIGHT] Mission complete. Landing..." << std::endl;
    offboard->stop();
    sleep_for(seconds(2));
    
    action->land();
    while (telemetry->in_air()) {
        sleep_for(seconds(1));
    }
    
    std::cout << "\n✨ --- MISSION COMPLETE ---\n" << std::endl;
    return 0;
}