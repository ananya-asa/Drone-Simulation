// #include "trajectory_follower.h"
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <cmath>

// using namespace mavsdk;
// using namespace std::this_thread;
// using namespace std::chrono;

// TrajectoryFollower::TrajectoryFollower(
//     std::shared_ptr<Telemetry> telemetry,
//     std::shared_ptr<Offboard> offboard,
//     const std::vector<Eigen::Vector3i>& path_grid,
//     double resolution)
//     : telemetry_(telemetry), offboard_(offboard), path_grid_(path_grid), resolution_(resolution) {}

// void TrajectoryFollower::start() {
//     std::cout << "TrajectoryFollower: Starting to follow " << path_grid_.size() << " waypoints" << std::endl;
    
//     for (size_t i = 0; i < path_grid_.size(); ++i) {
//         // FIX 6: USE ACTUAL Z VALUE FROM PATH, NOT HARDCODED -2.5
//         float north_m = path_grid_[i].x() * resolution_;
//         float east_m = path_grid_[i].y() * resolution_;
//         // Keep altitude CONSTANT at takeoff height (2.5m) for now
// // The planner's Z coordinate is for obstacle clearance, not actual altitude
// float down_m = -2.5f;  // Stay at takeoff altitude
//  // Convert grid Z to NED down
        
//         // Safety check: ensure altitude is reasonable (between 0.5m and 20m)
//         if (down_m > 20.0f) down_m = 20.0f;
//         if (down_m < 0.5f) down_m = 0.5f;
        
//         std::cout << "\nWaypoint " << i+1 << "/" << path_grid_.size()
//                   << ": Grid(" << path_grid_[i].x() << ","
//                   << path_grid_[i].y() << "," << path_grid_[i].z() << ")"
//                   << " -> NED(" << north_m << "," << east_m << "," << down_m << ")" << std::endl;
        
//         Offboard::PositionNedYaw setpoint{};
//         setpoint.north_m = north_m;
//         setpoint.east_m = east_m;
//         setpoint.down_m = down_m;
//         setpoint.yaw_deg = 0.0f;
        
//         offboard_->set_position_ned(setpoint);
//         sleep_for(milliseconds(200));
        
//         bool reached = false;
//         int timeout_count = 0;
//         const int max_timeout = 150;  // Increased from 100
//         const float distance_threshold = 0.3f;  // Increased from 0.15 for robustness
        
//         std::cout << " Waiting to reach waypoint...";
//         std::cout.flush();
        
//         while (!reached && timeout_count < max_timeout) {
//             Telemetry::PositionNed current_pos = telemetry_->position_velocity_ned().position;
            
//             float distance = std::sqrt(
//                 std::pow(current_pos.north_m - north_m, 2) +
//                 std::pow(current_pos.east_m - east_m, 2) +
//                 std::pow(current_pos.down_m - down_m, 2)
//             );
            
//             if (distance < distance_threshold) {
//                 reached = true;
//                 std::cout << " ✓ Reached (distance: " << distance << "m)" << std::endl;
//             }
            
//             sleep_for(milliseconds(100));
//             timeout_count++;
            
//             if (timeout_count % 15 == 0) {
//                 std::cout << ".";
//                 std::cout.flush();
//             }
//         }
        
//         if (!reached) {
//             std::cout << " ⚠ TIMEOUT (moved on after " << timeout_count 
//                       << " iterations - may indicate obstacle)" << std::endl;
//         }
        
//         sleep_for(milliseconds(500));
//     }
    
//     std::cout << "\nTrajectoryFollower: Path execution complete!" << std::endl;
// }


#include "trajectory_follower.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

TrajectoryFollower::TrajectoryFollower(
    std::shared_ptr<Telemetry> telemetry,
    std::shared_ptr<Offboard> offboard,
    const std::vector<Eigen::Vector3i>& path_grid,
    double resolution)
    : telemetry_(telemetry), offboard_(offboard), path_grid_(path_grid), resolution_(resolution) {}

void TrajectoryFollower::start() {
    std::cout << "TrajectoryFollower: Starting to follow " << path_grid_.size() << " waypoints" << std::endl;
    
    // FIX: Keep altitude CONSTANT at takeoff height (2.5m) during waypoint following
    // The grid Z coordinate is for obstacle clearance in planning, not actual flight altitude
    const float FLIGHT_ALTITUDE = -2.5f;  // 2.5m down = 2.5m altitude above ground
    
    for (size_t i = 0; i < path_grid_.size(); ++i) {
        // Get horizontal position from grid (X, Y only)
        float north_m = path_grid_[i].x() * resolution_;
        float east_m = path_grid_[i].y() * resolution_;
        
        // FIX: Use CONSTANT altitude, not grid Z
        float down_m = FLIGHT_ALTITUDE;
        
        std::cout << "\nWaypoint " << i+1 << "/" << path_grid_.size()
                  << ": Grid(" << path_grid_[i].x() << ","
                  << path_grid_[i].y() << "," << path_grid_[i].z() << ")"
                  << " -> NED(" << north_m << "," << east_m << "," << down_m << ")" << std::endl;
        
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = north_m;
        setpoint.east_m = east_m;
        setpoint.down_m = down_m;
        setpoint.yaw_deg = 0.0f;
        
        offboard_->set_position_ned(setpoint);
        
        // FIX: Increase command rate - send setpoints more frequently (50ms = 20Hz)
        // This prevents PX4 from timing out and triggering failsafe
        sleep_for(milliseconds(50));
        
        bool reached = false;
        int timeout_count = 0;
        const int max_timeout = 150;
        const float distance_threshold = 0.3f;
        
        std::cout << " Waiting to reach waypoint...";
        std::cout.flush();
        
        while (!reached && timeout_count < max_timeout) {
            // FIX: Check if drone is still in air - if landed, abort immediately
            if (!telemetry_->in_air()) {
                std::cout << " ⚠ DRONE LANDED - ABORTING WAYPOINT FOLLOWING" << std::endl;
                return;  // Exit trajectory following completely
            }
            
            Telemetry::PositionNed current_pos = telemetry_->position_velocity_ned().position;
            
            float distance = std::sqrt(
                std::pow(current_pos.north_m - north_m, 2) +
                std::pow(current_pos.east_m - east_m, 2) +
                std::pow(current_pos.down_m - down_m, 2)
            );
            
            if (distance < distance_threshold) {
                reached = true;
                std::cout << " ✓ Reached (distance: " << distance << "m)" << std::endl;
            }
            
            sleep_for(milliseconds(100));
            timeout_count++;
            
            if (timeout_count % 15 == 0) {
                std::cout << ".";
                std::cout.flush();
            }
        }
        
        if (!reached) {
            std::cout << " ⚠ TIMEOUT (moved on after " << timeout_count 
                      << " iterations - may indicate obstacle or communication issue)" << std::endl;
        }
        
        sleep_for(milliseconds(500));
    }
    
    std::cout << "\nTrajectoryFollower: Path execution complete!" << std::endl;
}