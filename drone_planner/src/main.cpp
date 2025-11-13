#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <future>
#include <atomic> // For std::atomic_bool

// MAVSDK Includes
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Our Project Includes
#include "path_planner.h"
#include "world_model.h"
#include "trajectory_follower.h" 

// Gazebo Bridge Includes
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

// PCL Includes (for data conversion)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

// --- Global variables for our live data ---
// We make these global so the callback can update them
// and the main loop can read them.
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::mutex cloud_mutex; // A "lock" to prevent bugs
std::atomic<bool> new_cloud_available(false); // A "flag"

// --- LIDAR CALLBACK FUNCTION (THE "LIVE EYES") ---
// This function will be called every time we get a LIDAR message
void lidar_callback(const gz::msgs::PointCloudPacked &msg) {
    
    // 1. Create a temporary cloud to hold the new data
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    temp_cloud->width = msg.width() * msg.height();
    temp_cloud->height = 1; // Unorganized point cloud
    temp_cloud->points.resize(temp_cloud->width);

    // 2. We must unpack the data
    // The data is "packed" as 'x', 'y', 'z', 'intensity'
    int point_step = msg.point_step();
    for (unsigned int i = 0; i < temp_cloud->width; ++i) {
        // We use 'memcpy' to copy the bytes
        memcpy(&temp_cloud->points[i].x, &msg.data()[i * point_step], sizeof(float));
        memcpy(&temp_cloud->points[i].y, &msg.data()[i * point_step + sizeof(float)], sizeof(float));
        memcpy(&temp_cloud->points[i].z, &msg.data()[i * point_step + 2 * sizeof(float)], sizeof(float));
    }
    
    // 3. Lock the "global" cloud and update it
    {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        lidar_cloud = temp_cloud;
    }
    
    // 4. Set the "flag" to tell the main loop there is new data
    new_cloud_available = true;
}
// ---

// --- MAVSDK get_system function (same as before) ---
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
    std::cout << "--- Autonomous Planner V3.0 (LIVE SENSOR) ---" << std::endl;

    // =======================================================
    // == PART 1: INITIALIZATION
    // =======================================================
    const double RESOLUTION = 0.2; // 20cm resolution (bigger for faster planning)
    WorldModel world(RESOLUTION);

    // --- Start the Sensor Bridge ---
    gz::transport::Node gz_node;
    if (!gz_node.Subscribe("/lidar", lidar_callback)) {
        std::cerr << "Failed to subscribe to /lidar topic" << std::endl;
        return 1;
    }
    std::cout << "Sensor Bridge created. Subscribed to /lidar." << std::endl;

    // --- Connect to the Drone ---
    Mavsdk mavsdk(Mavsdk::Configuration(mavsdk::ComponentType::GroundStation));

    ConnectionResult conn_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    if (conn_result != ConnectionResult::Success) { return 1; }

    auto system = get_system(mavsdk);
    if (!system) { return 1; }

    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    std::cout << "Waiting for drone to be ready to arm..." << std::endl;
    while (!telemetry->health_all_ok()) {
        std::cout << "Drone not healthy yet..." << std::endl;
        sleep_for(seconds(1));
    }

    // =======================================================
    // == PART 2: FLIGHT PHASE (THE "LIVE" LOOP)
    // =======================================================
    std::cout << "Arming drone..." << std::endl;
    action->arm();
    std::cout << "Taking off..." << std::endl;
    action->takeoff();
    sleep_for(seconds(5)); // Wait for takeoff

    std::cout << "Starting Offboard mode..." << std::endl;
    Offboard::PositionNedYaw stay_still{};
    stay_still.down_m = -2.5f; // Hold 2.5m altitude
    offboard->set_position_ned(stay_still);
    offboard->start();
    std::cout << "Offboard mode started!" << std::endl;

    // --- This is the new "live" loop ---
    for (int i = 0; i < 5; ++i) { // Let's just run 5 times for this test
        std::cout << "\n--- PLANNING LOOP " << i+1 << " ---" << std::endl;
        
        // 1. Wait for new sensor data
        while(!new_cloud_available) {
            sleep_for(milliseconds(10));
        }

        // 2. Get the new map
        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            world.buildMap(lidar_cloud); // Build map from *live* data
        }
        new_cloud_available = false; // Reset the flag

        // 3. Get our current position (as a grid cell)
        Telemetry::PositionNed current_pos = telemetry->position_velocity_ned().position;
        Eigen::Vector3i start_grid(
            std::round(current_pos.north_m / RESOLUTION),
            std::round(current_pos.east_m / RESOLUTION),
            0 // We'll plan in 2D for now
        );

        // 4. Set a goal
        Eigen::Vector3i goal_grid(10, 5, 0); // 10m North, 5m East
        
        std::cout << "Finding path from " << start_grid.transpose() 
                  << " to " << goal_grid.transpose() << "..." << std::endl;

        // 5. Plan the path
        std::vector<Eigen::Vector3i> path_grid = A_star_search(start_grid, goal_grid, world);

        if (path_grid.empty()) {
            std::cerr << "PLANNING FAILED: No path was found." << std::endl;
            sleep_for(seconds(1));
            continue; // Try again
        }
        
        std::cout << "PLAN SUCCESS! Path found with " << path_grid.size() << " steps." << std::endl;

        // 6. Fly the path
        TrajectoryFollower follower(telemetry, offboard, path_grid, RESOLUTION);
        follower.start(); // This will fly the new path
    }
    // --- End of live loop ---

    std::cout << "Stopping Offboard mode and landing..." << std::endl;
    offboard->stop();
    action->land();
    
    while (telemetry->in_air()) {
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;
    std::cout << "\n[SUCCESS] V3.0 LIVE SENSOR TEST COMPLETE." << std::endl;

    return 0;
}