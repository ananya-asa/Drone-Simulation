#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <future>

// MAVSDK Includes
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Our Project Includes
#include "path_planner.h"
#include "world_model.h"
#include "trajectory_follower.h"

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

// Helper function to create static test wall
pcl::PointCloud<pcl::PointXYZ>::Ptr create_test_wall() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Wall at x=0.5m
    for (double y = -1.0; y <= 1.0; y += 0.1) {
        for (double z = -1.0; z <= 1.0; z += 0.1) {
            cloud->points.push_back(pcl::PointXYZ(0.5, y, z));
        }
    }
    std::cout << "Created test wall with " << cloud->points.size() << " points" << std::endl;
    return cloud;
}

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
    std::cout << "=== Autonomous Drone Planner V3.0 ===" << std::endl;

    // Setup world model with static obstacle
    const double RESOLUTION = 0.2;
    WorldModel world(RESOLUTION);
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall = create_test_wall();
    world.buildMap(wall);

    // Define start and goal in grid coordinates
    Eigen::Vector3i start_grid(0, 0, 0);
    Eigen::Vector3i goal_grid(10, 0, 0); // 10 * 0.2m = 2m forward

    std::cout << "\nPlanning path from (0,0,0) to (10,0,0) in grid space" << std::endl;
    std::cout << "Wall is at x=0.5m (grid x=5)" << std::endl;

    std::vector<Eigen::Vector3i> path_grid = A_star_search(start_grid, goal_grid, world);
    
    if (path_grid.empty()) {
        std::cerr << "ERROR: No path found!" << std::endl;
        return 1;
    }

    std::cout << "SUCCESS: Path found with " << path_grid.size() << " waypoints" << std::endl;

    // Connect to PX4 SITL
    Mavsdk mavsdk(Mavsdk::Configuration(mavsdk::ComponentType::GroundStation));
    ConnectionResult conn_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    
    if (conn_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << conn_result << std::endl;
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) { return 1; }

    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    // Wait for healthy state
    std::cout << "Waiting for drone to be ready..." << std::endl;
    while (!telemetry->health_all_ok()) {
        sleep_for(seconds(1));
    }

    // Arm and takeoff
    std::cout << "Arming..." << std::endl;
    if (action->arm() != Action::Result::Success) {
        std::cerr << "Arming failed!" << std::endl;
        return 1;
    }

    std::cout << "Taking off..." << std::endl;
    if (action->takeoff() != Action::Result::Success) {
        std::cerr << "Takeoff failed!" << std::endl;
        return 1;
    }
    sleep_for(seconds(5));

    // Start offboard mode
    std::cout << "Starting Offboard mode..." << std::endl;
    Offboard::PositionNedYaw initial_setpoint{};
    initial_setpoint.north_m = 0.0f;
    initial_setpoint.east_m = 0.0f;
    initial_setpoint.down_m = -2.5f;
    initial_setpoint.yaw_deg = 0.0f;
    
    offboard->set_position_ned(initial_setpoint);
    
    if (offboard->start() != Offboard::Result::Success) {
        std::cerr << "Offboard start failed!" << std::endl;
        return 1;
    }
    
    sleep_for(seconds(2));
    std::cout << "Offboard mode active!" << std::endl;

    // Execute trajectory
    std::cout << "\n=== Executing Planned Path ===" << std::endl;
    TrajectoryFollower follower(telemetry, offboard, path_grid, RESOLUTION);
    follower.start();

    // Land
    std::cout << "\n=== Landing ===" << std::endl;
    offboard->stop();
    sleep_for(seconds(2));
    action->land();

    while (telemetry->in_air()) {
        std::cout << "Landing..." << std::endl;
        sleep_for(seconds(1));
    }

    std::cout << "\n=== MISSION COMPLETE ===" << std::endl;
    return 0;
}
