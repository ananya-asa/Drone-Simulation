#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <future>

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Our headers
#include "path_planner.h"
#include "world_model.h"
#include "trajectory_follower.h"
#include "lidar_bridge.h"

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

std::shared_ptr<System> get_system(Mavsdk& mavsdk) {
    std::cout << "Waiting to discover drone..." << std::endl;
    auto prom = std::promise<std::shared_ptr<System>>();
    auto fut = prom.get_future();
    Mavsdk::NewSystemHandle handle;
    handle = mavsdk.subscribe_on_new_system(
        [&mavsdk, &prom, &handle]() {
            auto system = mavsdk.systems().at(0);
            if (system->has_autopilot()) {
                std::cout << "Discovered autopilot!" << std::endl;
                mavsdk.unsubscribe_on_new_system(handle);
                prom.set_value(system);
            }
        });
    if (fut.wait_for(seconds(10)) == std::future_status::timeout) {
        std::cerr << "No drone found, timeout!" << std::endl;
        return nullptr;
    }
    return fut.get();
}

int main() {
    std::cout << "\n=== Autonomous Drone Planner V4.0 (LIVE LIDAR) ===" << std::endl;

    // Setup world model
    const double RESOLUTION = 0.2;
    WorldModel world(RESOLUTION);

    // Setup LIDAR bridge
    std::cout << "\n[SETUP] Initializing LIDAR sensor bridge..." << std::endl;
    LIDARBridge lidar;
    if (!lidar.subscribe()) {
        std::cerr << "[ERROR] Failed to subscribe to LIDAR" << std::endl;
        return 1;
    }

    // Give sensor time to start publishing
    std::cout << "[SETUP] Waiting 3 seconds for LIDAR to start publishing..." << std::endl;
    sleep_for(seconds(3));

    // Connect to PX4 SITL
    std::cout << "\n[SETUP] Connecting to PX4 SITL..." << std::endl;
    Mavsdk mavsdk(Mavsdk::Configuration(mavsdk::ComponentType::GroundStation));
    ConnectionResult conn_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    
    if (conn_result != ConnectionResult::Success) {
        std::cerr << "[ERROR] Connection failed: " << conn_result << std::endl;
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) { return 1; }

    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    // Wait for drone health
    std::cout << "\n[SETUP] Waiting for drone to be ready..." << std::endl;
    int health_wait_count = 0;
    while (!telemetry->health_all_ok() && health_wait_count < 30) {
        sleep_for(seconds(1));
        health_wait_count++;
    }

    if (!telemetry->health_all_ok()) {
        std::cout << "[WARNING] Drone health check incomplete, continuing anyway..." << std::endl;
    }

    // === MAIN PLANNING LOOP ===
    std::cout << "\n[MISSION] Starting autonomous mission with live LIDAR..." << std::endl;

    for (int mission = 0; mission < 3; ++mission) {
        std::cout << "\n--- MISSION " << mission + 1 << " ---" << std::endl;

        // Wait for LIDAR data (max 5 seconds)
        std::cout << "[LIDAR] Waiting for sensor data..." << std::endl;
        int wait_count = 0;
        while (!lidar.has_new_data() && wait_count < 50) {
            sleep_for(milliseconds(100));
            wait_count++;
        }

        if (!lidar.has_new_data()) {
            std::cout << "[WARNING] No LIDAR data received yet, using fallback..." << std::endl;
            // Fallback: use static wall (optional)
            pcl::PointCloud<pcl::PointXYZ>::Ptr fallback(new pcl::PointCloud<pcl::PointXYZ>);
            for (double y = -1.0; y <= 1.0; y += 0.1) {
                for (double z = -1.0; z <= 1.0; z += 0.1) {
                    fallback->points.push_back(pcl::PointXYZ(0.5, y, z));
                }
            }
            world.buildMap(fallback);
        } else {
            std::cout << "[LIDAR] Data received! Building map with " 
                      << lidar.get_cloud()->points.size() << " points" << std::endl;
            world.buildMap(lidar.get_cloud());
            lidar.reset_data_flag();
        }

        // Plan path
        Eigen::Vector3i start_grid(0, 0, 0);
        Eigen::Vector3i goal_grid(10, 0, 0);

        std::cout << "[PLANNING] Planning path from " << start_grid.transpose() 
                  << " to " << goal_grid.transpose() << std::endl;

        std::vector<Eigen::Vector3i> path_grid = A_star_search(start_grid, goal_grid, world);

        if (path_grid.empty()) {
            std::cerr << "[ERROR] No path found!" << std::endl;
            continue;
        }

        std::cout << "[PLANNING] Success! Path has " << path_grid.size() << " waypoints" << std::endl;

        // Only execute flight on first mission
        if (mission == 0) {
            std::cout << "\n[FLIGHT] Arming drone..." << std::endl;
            if (action->arm() != Action::Result::Success) {
                std::cerr << "[ERROR] Arming failed!" << std::endl;
                return 1;
            }

            std::cout << "[FLIGHT] Taking off..." << std::endl;
            if (action->takeoff() != Action::Result::Success) {
                std::cerr << "[ERROR] Takeoff failed!" << std::endl;
                return 1;
            }
            sleep_for(seconds(5));

            std::cout << "[FLIGHT] Starting Offboard mode..." << std::endl;
            Offboard::PositionNedYaw initial_setpoint{};
            initial_setpoint.north_m = 0.0f;
            initial_setpoint.east_m = 0.0f;
            initial_setpoint.down_m = -2.5f;
            initial_setpoint.yaw_deg = 0.0f;
            
            offboard->set_position_ned(initial_setpoint);
            
            if (offboard->start() != Offboard::Result::Success) {
                std::cerr << "[ERROR] Offboard start failed!" << std::endl;
                return 1;
            }
            sleep_for(seconds(2));

            std::cout << "[FLIGHT] Executing trajectory..." << std::endl;
            TrajectoryFollower follower(telemetry, offboard, path_grid, RESOLUTION);
            follower.start();

            std::cout << "\n[FLIGHT] Landing..." << std::endl;
            offboard->stop();
            sleep_for(seconds(2));
            action->land();

            while (telemetry->in_air()) {
                sleep_for(seconds(1));
            }
        }
    }

    std::cout << "\n=== MISSION COMPLETE ===" << std::endl;
    std::cout << "[STATS] Total LIDAR callbacks: " << lidar.get_callback_count() << std::endl;
    return 0;
}
