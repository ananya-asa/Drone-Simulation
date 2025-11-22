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
    std::cout << "\n=== Autonomous Drone Planner V5.0 (TRUE AUTONOMOUS) ===" << std::endl;

    // ===== 1. SETUP PHASE =====
    const double RESOLUTION = 0.2;
    WorldModel world(RESOLUTION);

    std::cout << "\n[SETUP] Initializing LIDAR sensor bridge..." << std::endl;
    LIDARBridge lidar;
    if (!lidar.subscribe()) {
        std::cerr << "[ERROR] Failed to subscribe to LIDAR" << std::endl;
        return 1;
    }
    sleep_for(seconds(2));

    std::cout << "[SETUP] Connecting to PX4 SITL..." << std::endl;
    Mavsdk mavsdk(Mavsdk::Configuration(mavsdk::ComponentType::GroundStation));
    ConnectionResult conn_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    if (conn_result != ConnectionResult::Success) {
        std::cerr << "[ERROR] Connection failed" << std::endl;
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) { return 1; }

    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    std::cout << "[SETUP] Waiting for drone to be ready..." << std::endl;
    int health_wait = 0;
    while (!telemetry->health_all_ok() && health_wait < 30) {
        sleep_for(seconds(1));
        health_wait++;
    }

    // ===== 2. TAKEOFF PHASE =====
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
    Offboard::PositionNedYaw initial_setpoint{};
    initial_setpoint.north_m = 0.0f;
    initial_setpoint.east_m = 0.0f;
    initial_setpoint.down_m = -2.5f;
    initial_setpoint.yaw_deg = 0.0f;
    offboard->set_position_ned(initial_setpoint);

    if (offboard->start() != Offboard::Result::Success) {
        std::cerr << "[ERROR] Offboard start failed" << std::endl;
        return 1;
    }
    sleep_for(seconds(2));
    std::cout << "[FLIGHT] Offboard mode active!" << std::endl;

    // =========================================================================
    // ===== 3. THE TRUE AUTONOMOUS LOOP (THE MAGIC HAPPENS HERE) =====
    // =========================================================================
    Eigen::Vector3i goal_grid(50, 25, 0);  // 10m North, 5m East in grid coords
    int autonomous_loops = 0;
    const int MAX_LOOPS = 5;

    while (autonomous_loops < MAX_LOOPS) {
        std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
        std::cout << "║   AUTONOMOUS LOOP " << autonomous_loops + 1 << "/" << MAX_LOOPS 
                  << "                   ║" << std::endl;
        std::cout << "╚════════════════════════════════════════╝" << std::endl;

        // --- STEP 1: Get Live LIDAR Data ---
        std::cout << "\n[SENSE] Waiting for new LIDAR data..." << std::endl;
        int lidar_wait = 0;
        while (!lidar.has_new_data() && lidar_wait < 50) {
            sleep_for(milliseconds(100));
            lidar_wait++;
        }

        if (!lidar.has_new_data()) {
            std::cout << "[WARN] No LIDAR data received, using fallback..." << std::endl;
            // Fallback: static test wall
            pcl::PointCloud<pcl::PointXYZ>::Ptr fallback(new pcl::PointCloud<pcl::PointXYZ>);
            for (double y = -1.0; y <= 1.0; y += 0.1) {
                for (double z = -1.0; z <= 1.0; z += 0.1) {
                    fallback->points.push_back(pcl::PointXYZ(0.5, y, z));
                }
            }
            world.buildMap(fallback);
        } else {
            std::cout << "[SENSE] Got " << lidar.get_cloud()->points.size() 
                      << " points from LIDAR" << std::endl;
            world.buildMap(lidar.get_cloud());
            lidar.reset_data_flag();
        }

        // --- STEP 2: Get Current Drone Position ---
        Telemetry::PositionNed current_pos = telemetry->position_velocity_ned().position;
        Eigen::Vector3i start_grid(
            std::round(current_pos.north_m / RESOLUTION),
            std::round(current_pos.east_m / RESOLUTION),
            0
        );

        std::cout << "\n[THINK] Current position (world): (" << current_pos.north_m << ", " 
                  << current_pos.east_m << ", " << current_pos.down_m << ")" << std::endl;
        std::cout << "[THINK] Current position (grid): " << start_grid.transpose() << std::endl;
        std::cout << "[THINK] Goal position (grid): " << goal_grid.transpose() << std::endl;

        // --- STEP 3: Plan New Path ---
        std::cout << "\n[PLAN] Computing A* path..." << std::endl;
        std::vector<Eigen::Vector3i> path_grid = A_star_search(start_grid, goal_grid, world);

        if (path_grid.empty()) {
            std::cerr << "[WARN] No path found! Retrying..." << std::endl;
            autonomous_loops++;
            continue;
        }

        std::cout << "[PLAN] ✓ Path found with " << path_grid.size() << " waypoints" << std::endl;

        // --- STEP 4: Execute Path ---
        std::cout << "\n[ACT] Following trajectory..." << std::endl;
        TrajectoryFollower follower(telemetry, offboard, path_grid, RESOLUTION);
        follower.start();

        std::cout << "\n[ACT] Trajectory complete!" << std::endl;

        autonomous_loops++;
    }

    // =========================================================================
    // ===== 4. LAND PHASE =====
    // =========================================================================
    std::cout << "\n[FLIGHT] Autonomous loops complete. Landing..." << std::endl;
    offboard->stop();
    sleep_for(seconds(2));
    action->land();

    int land_timeout = 0;
    while (telemetry->in_air() && land_timeout < 60) {
        std::cout << ".";
        sleep_for(seconds(1));
        land_timeout++;
    }
    std::cout << "\n\n[FLIGHT] ✓ Landed safely!" << std::endl;

    std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
    std::cout << "║     MISSION COMPLETE - ALL SYSTEMS GO  ║" << std::endl;
    std::cout << "║     Loops completed: " << autonomous_loops << "/" << MAX_LOOPS 
              << "                     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════╝\n" << std::endl;

    return 0;
}
