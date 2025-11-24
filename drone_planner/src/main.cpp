#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <future>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

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
    
    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
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

    const double RESOLUTION = 0.2;
    WorldModel world(RESOLUTION);

    std::cout << "\n[SETUP] Initializing LIDAR sensor bridge..." << std::endl;
    LIDARBridge lidar;
    if (!lidar.subscribe()) {
        std::cerr << "[ERROR] Failed to subscribe to LIDAR" << std::endl;
        return 1;
    }

    std::cout << "[SETUP] Waiting 3 seconds for LIDAR to start publishing..." << std::endl;
    sleep_for(seconds(3));

    std::cout << "\n[SETUP] Connecting to PX4 SITL..." << std::endl;
    
    // FIX: Use the correct constructor for newer MAVSDK
    Mavsdk mavsdk(Mavsdk::Configuration(ComponentType::GroundStation));
    
    ConnectionResult connection_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "[ERROR] Connection failed: " << connection_result << std::endl;
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    // FIX: Use std::shared_ptr<System> constructor
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    std::cout << "\n[SETUP] Waiting for drone to be ready..." << std::endl;
    while (!telemetry->health_all_ok()) {
        sleep_for(seconds(1));
    }

    std::cout << "\n[MISSION] Starting autonomous mission with live LIDAR..." << std::endl;

    for (int mission = 0; mission < 3; mission++) {
        std::cout << "\n--- MISSION " << mission + 1 << " ---" << std::endl;

        std::cout << "[LIDAR] Waiting for sensor data..." << std::endl;
        int wait_count = 0;
        while (!lidar.has_new_data() && wait_count < 30) {
            sleep_for(milliseconds(100));
            wait_count++;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (lidar.has_new_data()) {
            cloud = lidar.get_cloud();
            lidar.reset_data_flag();
            std::cout << "[LIDAR] Got " << cloud->points.size() << " points" << std::endl;
        } else {
            std::cout << "[WARNING] No LIDAR data received yet, using fallback..." << std::endl;
            cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for (double y = -1.0; y <= 1.0; y += 0.1) {
                for (double z = -1.0; z <= 1.0; z += 0.1) {
                    cloud->points.push_back(pcl::PointXYZ(0.5, y, z));
                }
            }
        }

        std::cout << "Building map with " << cloud->points.size() << " points..." << std::endl;
        world.buildMap(cloud);
        std::cout << "Map built." << std::endl;

        Eigen::Vector3i start_grid(0, 0, 0);
        Eigen::Vector3i goal_grid(10, 0, 0);

        std::cout << "[PLANNING] Planning path from " << start_grid.transpose() 
                  << " to " << goal_grid.transpose() << std::endl;

        std::vector<Eigen::Vector3i> path_grid = A_star_search(start_grid, goal_grid, world);

        if (path_grid.empty()) {
            std::cerr << "[PLANNING] No path found!" << std::endl;
            continue;
        }

        std::cout << "[PLANNING] Success! Path has " << path_grid.size() << " waypoints" << std::endl;

        if (mission == 0) {
            std::cout << "\n[FLIGHT] Arming drone..." << std::endl;
            const Action::Result arm_result = action->arm();
            if (arm_result != Action::Result::Success) {
                std::cerr << "[ERROR] Arming failed" << std::endl;
                return 1;
            }

            std::cout << "[FLIGHT] Taking off..." << std::endl;
            const Action::Result takeoff_result = action->takeoff();
            if (takeoff_result != Action::Result::Success) {
                std::cerr << "[ERROR] Takeoff failed" << std::endl;
                return 1;
            }
            sleep_for(seconds(5));

            std::cout << "[FLIGHT] Starting Offboard mode..." << std::endl;
            Offboard::PositionNedYaw stay{};
            stay.north_m = 0.0f;
            stay.east_m = 0.0f;
            stay.down_m = -2.5f;
            stay.yaw_deg = 0.0f;
            offboard->set_position_ned(stay);

            Offboard::Result offboard_result = offboard->start();
            if (offboard_result != Offboard::Result::Success) {
                std::cerr << "[ERROR] Offboard start failed" << std::endl;
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
    std::cout << "[STATS] Total LIDAR callbacks: " << 0 << std::endl;

    return 0;
}
