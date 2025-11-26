#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <future>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "path_planner.h"
#include "world_model.h"
#include "trajectory_follower.h"
#include "lidar_bridge.h"
#include <Eigen/Dense>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>

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
            std::cout << "[LOG] Flight data will be saved to: " << filename << std::endl;
        }
    }
    ~FlightLogger() {
        if (log_file.is_open()) {
            log_file.close();
            std::cout << "[LOG] Flight data saved successfully." << std::endl;
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

#include <future>
using namespace mavsdk;

std::shared_ptr<System> get_system(Mavsdk& mavsdk) {
    std::cout << "Waiting to discover drone..." << std::endl;
    auto prom = std::promise<std::shared_ptr<System>>();
    auto fut = prom.get_future();

    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().at(0);
        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot!" << std::endl;
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        std::cerr << "No drone found, timeout!" << std::endl;
        return nullptr;
    }
    return fut.get();
}

// ---- Paste get_system and FlightLogger class code above main as previously given ----

int main() {
    std::cout << "\n=== Autonomous Drone Planner V5.2 (LOGGING + INTERACTIVE) ===" << std::endl;

    const double RESOLUTION = 0.2;
    WorldModel world(RESOLUTION);
    FlightLogger logger("flight_log.csv");
    LIDARBridge lidar;

    // ---- MAVSDK Setup ----
    Mavsdk mavsdk(Mavsdk::Configuration(ComponentType::GroundStation));
    ConnectionResult connection_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "[ERROR] Connection failed: " << connection_result << std::endl;
        return 1;
    }
    auto system = get_system(mavsdk);
    if (!system) return 1;
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    // ---- User Input Goal and DEBUG OUTPUT for Altitude ----
    std::cout << "\n[INPUT] Enter goal position in NED coordinates:" << std::endl;
    std::cout << "  North (meters): "; float goal_north; std::cin >> goal_north;
    std::cout << "  East (meters): "; float goal_east; std::cin >> goal_east;
    std::cout << "  Down (meters, put -2.5 for 2.5m up): "; float goal_down; std::cin >> goal_down;
    Eigen::Vector3d goal_ned(goal_north, goal_east, goal_down);

    // ------- ALTITUDE/GRID CONVERSION FIX -------    
    Eigen::Vector3i goal_grid = world.worldToGrid(goal_ned); // Uses corrected method from world_model.cpp
    std::cout << "[DEBUG] Goal NED: " << goal_ned.transpose() << std::endl;
    std::cout << "[DEBUG] Goal Grid: " << goal_grid.transpose() << " (should have positive z for altitude!)" << std::endl;

    // ---- Lidar Setup ----
    std::cout << "\n[SETUP] Initializing LIDAR sensor bridge..." << std::endl;
    if (!lidar.subscribe()) {
        std::cerr << "[ERROR] Failed to subscribe to LIDAR" << std::endl;
        return 1;
    }
    sleep_for(seconds(3));

    std::cout << "\n[SETUP] Waiting for drone to be ready..." << std::endl;
    while (!telemetry->health_all_ok()) sleep_for(seconds(1));

    std::cout << "\n[FLIGHT] Arming drone..." << std::endl;
    if (action->arm() != Action::Result::Success) {
        std::cerr << "[ERROR] Arming failed" << std::endl;
        return 1;
    }

    std::cout << "[FLIGHT] Taking off..." << std::endl;
    if (action->takeoff() != Action::Result::Success) {
        std::cerr << "[ERROR] Takeoff failed" << std::endl;
        return 1;
    }
    sleep_for(seconds(5));

    std::cout << "[FLIGHT] Starting Offboard mode..." << std::endl;
    Offboard::PositionNedYaw stay{};
    stay.north_m = 0.0f; stay.east_m = 0.0f; stay.down_m = -2.5f; stay.yaw_deg = 0.0f;
    offboard->set_position_ned(stay);
    if (offboard->start() != Offboard::Result::Success) {
        std::cerr << "[ERROR] Offboard start failed" << std::endl;
        return 1;
    }
    sleep_for(seconds(2));

    std::cout << "\n[MISSION] Starting dynamic replanning mission..." << std::endl;

    const double GOAL_TOLERANCE = 0.5;
    const int MAX_CYCLES = 50;
    const int MAX_FAILS = 5;
    int fail_count = 0;

    for (int cycle = 0; cycle < MAX_CYCLES; ++cycle) {
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
            cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for (double y = -1.0; y <= 1.0; y += 0.1)
                for (double z = -1.0; z <= 1.0; z += 0.1)
                    cloud->points.push_back(pcl::PointXYZ(0.5, y, z));
        }
        world.buildMap(cloud);

        // ------ FIX USE GRID ALTITUDE EVERYWHERE ------
        Telemetry::PositionNed pos = telemetry->position_velocity_ned().position;
        Eigen::Vector3d start_ned(pos.north_m, pos.east_m, pos.down_m);
        Eigen::Vector3i start_grid = world.worldToGrid(start_ned);

        std::cout << "[DEBUG] Start NED: " << start_ned.transpose() << std::endl;
        std::cout << "[DEBUG] Start Grid: " << start_grid.transpose() << std::endl;

        auto path_grid = A_star_search(start_grid, goal_grid, world);

        if (path_grid.empty()) {
            std::cerr << "[PLAN] ✗ No path found! Hovering..." << std::endl;
            logger.log_cycle(cycle + 1, pos, start_grid, goal_grid, -1.0f, 0, cloud->points.size(), "NO_PATH", 0);
            fail_count++;
            if (fail_count >= MAX_FAILS) {
                std::cerr << "[PLAN] ❌ Maximum replanning failures exceeded. Landing." << std::endl;
                break;
            }
            continue;
        }
        fail_count = 0;

        std::cout << "[PLAN] ✓ Path found: " << path_grid.size() << " waypoints" << std::endl;
        int seg_end = std::min(3, static_cast<int>(path_grid.size()));
        std::vector<Eigen::Vector3i> this_leg(path_grid.begin(), path_grid.begin() + seg_end);
        std::cout << "[ACT] Flying next " << seg_end << " waypoints..." << std::endl;
        TrajectoryFollower follower(telemetry, offboard, this_leg, RESOLUTION);
        follower.start();

        Eigen::Vector3f distance_vec = (path_grid.back().cast<float>() - goal_grid.cast<float>()) * RESOLUTION;
        float distance_to_goal = distance_vec.norm();

        std::cout << "[STATUS] Distance to goal: " << distance_to_goal << "m" << std::endl;
        logger.log_cycle(cycle + 1, pos, start_grid, goal_grid, distance_to_goal,
                   path_grid.size(), cloud->points.size(), "SUCCESS", seg_end);

        if (distance_to_goal < GOAL_TOLERANCE) {
            std::cout << "\n[GOAL REACHED]\n" << std::endl;
            break;
        }
    }

    std::cout << "\n[FLIGHT] Mission complete. Landing..." << std::endl;
    offboard->stop();
    sleep_for(seconds(2));
    action->land();
    while (telemetry->in_air()) sleep_for(seconds(1));
    std::cout << "\n--- MISSION COMPLETE ---\n" << std::endl;
    return 0;
}
