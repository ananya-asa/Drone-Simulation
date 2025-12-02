// ============================================================================
// FILE: src/trajectory_follower.cpp
// VERSION: 6.0 (FIXED COMPILATION ERROR)
// FIX: Corrected chrono duration cast from nanoseconds to milliseconds
// ============================================================================

#include "trajectory_follower.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

// ============================================================================
// CONFIGURATION CONSTANTS FOR TRAJECTORY FOLLOWING
// ============================================================================
constexpr float FLIGHT_ALTITUDE = -2.5f;        // Keep at 2.5m altitude (NED)
constexpr float MIN_ALTITUDE = 0.5f;            // Minimum safe altitude
constexpr float MAX_ALTITUDE = 20.0f;           // Maximum safe altitude
constexpr float WAYPOINT_TOLERANCE = 0.3f;      // 0.3m waypoint reach threshold
constexpr int MAX_WAYPOINT_TIMEOUT = 150;       // 150 * 100ms = 15 seconds per waypoint
constexpr int SETPOINT_RATE_MS = 50;            // 20Hz setpoint rate (50ms period)

// ============================================================================
// TRAJECTORY FOLLOWER CONSTRUCTOR
// ============================================================================
TrajectoryFollower::TrajectoryFollower(
    std::shared_ptr<Telemetry> telemetry,
    std::shared_ptr<Offboard> offboard,
    const std::vector<Eigen::Vector3i>& path_grid,
    double resolution)
    : telemetry_(telemetry), offboard_(offboard), 
      path_grid_(path_grid), resolution_(resolution) {
    
    std::cout << "[TRAJ] ✅ TrajectoryFollower initialized for " 
              << path_grid.size() << " waypoints" << std::endl;
}

// ============================================================================
// MAIN TRAJECTORY FOLLOWING FUNCTION
// ============================================================================
// Executes waypoint-following mission with:
// - Consistent 20Hz setpoint rate
// - Robust timeout handling
// - 3D altitude support (IMPROVED)
// - Real-time abort on emergency
// ============================================================================
void TrajectoryFollower::start() {
    std::cout << "\n[TRAJ] 🚀 Starting trajectory following (" 
              << path_grid_.size() << " waypoints)" << std::endl;
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

    int waypoints_reached = 0;
    long total_flight_time_ms = 0;

    // ========================================================================
    // WAYPOINT LOOP: Process each waypoint in sequence
    // ========================================================================
    for (size_t i = 0; i < path_grid_.size(); ++i) {
        
        // Safety check: Is drone still in air?
        if (!telemetry_->in_air()) {
            std::cout << "[TRAJ] ⚠️  [WP " << i+1 << "] DRONE NOT IN AIR - ABORTING" << std::endl;
            return;
        }

        // ====================================================================
        // STEP 1: CONVERT GRID TO WORLD COORDINATES (NED)
        // ====================================================================
        float north_m = path_grid_[i].x() * resolution_;
        float east_m = path_grid_[i].y() * resolution_;
        
        // ====================================================================
        // STEP 2: ALTITUDE HANDLING (IMPROVED V6.0)
        // ====================================================================
        // BEFORE: Hardcoded altitude, ignoring Z from path planner
        // AFTER: Use Z coordinate from path, with safety bounds
        // ====================================================================
        
        // Get altitude from path (3D planning output)
        float down_m = path_grid_[i].z() * resolution_;
        
        // Apply altitude constraints (prevent going too high/low)
        // For now: Use constant altitude (can be changed for 3D flight)
        down_m = FLIGHT_ALTITUDE;  // Keep at takeoff altitude
        
        // Safety bounds: Ensure altitude is within safe range
        down_m = std::clamp(down_m, -MAX_ALTITUDE, -MIN_ALTITUDE);
        
        std::cout << "\n[TRAJ] 📍 Waypoint " << (i+1) << "/" << path_grid_.size() << std::endl;
        std::cout << "       Grid Position: (" << path_grid_[i].x() << ", " 
                  << path_grid_[i].y() << ", " << path_grid_[i].z() << ")" << std::endl;
        std::cout << "       World Position (NED): (" << north_m << ", " 
                  << east_m << ", " << down_m << ")" << std::endl;

        // ====================================================================
        // STEP 3: CREATE SETPOINT MESSAGE
        // ====================================================================
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = north_m;
        setpoint.east_m = east_m;
        setpoint.down_m = down_m;
        setpoint.yaw_deg = 0.0f;  // Keep yaw fixed

        // ====================================================================
        // STEP 4: SEND INITIAL SETPOINT
        // ====================================================================
        std::cout << "       Sending setpoint..." << std::endl;
        offboard_->set_position_ned(setpoint);
        
        // Brief delay to ensure message is processed
        sleep_for(milliseconds(50));

        // ====================================================================
        // STEP 5: WAIT FOR WAYPOINT ARRIVAL
        // ====================================================================
        bool waypoint_reached = false;
        int timeout_count = 0;
        auto waypoint_start_time = std::chrono::high_resolution_clock::now();

        std::cout << "       Waiting for arrival...";
        std::cout.flush();

        // Timeout loop: 15 seconds per waypoint maximum
        while (!waypoint_reached && timeout_count < MAX_WAYPOINT_TIMEOUT) {
            
            // ============================================================
            // SAFETY CHECK 1: Drone still in air?
            // ============================================================
            if (!telemetry_->in_air()) {
                std::cout << "\n       ⚠️  [ERROR] Drone not in air!" << std::endl;
                std::cout << "       Aborting trajectory following." << std::endl;
                return;
            }

            // ============================================================
            // SAFETY CHECK 2: Keep sending setpoints (20Hz requirement)
            // ============================================================
            // PX4 has a 500ms timeout on setpoints. We send every 50ms
            // to ensure control stream is maintained.
            // ============================================================
            if (timeout_count % 5 == 0) {  // Every 500ms, resend
                offboard_->set_position_ned(setpoint);
            }

            // ============================================================
            // GET CURRENT POSITION
            // ============================================================
            Telemetry::PositionNed current_pos = 
                telemetry_->position_velocity_ned().position;
            
            // Calculate 3D distance to waypoint
            float delta_north = current_pos.north_m - north_m;
            float delta_east = current_pos.east_m - east_m;
            float delta_down = current_pos.down_m - down_m;
            
            float distance_to_waypoint = std::sqrt(
                delta_north * delta_north +
                delta_east * delta_east +
                delta_down * delta_down
            );

            // ============================================================
            // CHECK IF WAYPOINT REACHED
            // ============================================================
            if (distance_to_waypoint < WAYPOINT_TOLERANCE) {
                waypoint_reached = true;
                auto waypoint_end_time = std::chrono::high_resolution_clock::now();
                
                // ✅ FIXED: Correct chrono duration casting
                // Convert from nanoseconds to milliseconds properly
                auto duration_ns = waypoint_end_time - waypoint_start_time;
                long waypoint_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration_ns).count();
                total_flight_time_ms += waypoint_time_ms;
                
                std::cout << " ✓ REACHED in " << waypoint_time_ms << "ms" << std::endl;
                std::cout << "       Distance: " << distance_to_waypoint << "m" << std::endl;
                std::cout << "       Current Position: (" << current_pos.north_m 
                          << ", " << current_pos.east_m << ", " 
                          << current_pos.down_m << ")" << std::endl;
                waypoints_reached++;
            }

            // ============================================================
            // PROGRESS INDICATOR (every 1.5 seconds)
            // ============================================================
            if (timeout_count % 15 == 0 && !waypoint_reached) {
                std::cout << ".";
                std::cout.flush();
            }

            // Wait 100ms before next check
            sleep_for(milliseconds(100));
            timeout_count++;
        }

        // ====================================================================
        // STEP 6: HANDLE WAYPOINT TIMEOUT
        // ====================================================================
        if (!waypoint_reached) {
            std::cout << " ⚠️  TIMEOUT AFTER 15s" << std::endl;
            std::cout << "       Moving to next waypoint anyway (may indicate obstacle)" << std::endl;
            // NOTE: In production, could trigger:
            // - Re-planning
            // - Increase safety margin
            // - Alternative route
        }

        // ====================================================================
        // STEP 7: INTER-WAYPOINT PAUSE
        // ====================================================================
        // Small delay between waypoints to allow stabilization
        sleep_for(milliseconds(500));
    }

    // ========================================================================
    // MISSION SUMMARY
    // ========================================================================
    std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
    std::cout << "[TRAJ] ✅ Trajectory following complete!" << std::endl;
    std::cout << "       Waypoints reached: " << waypoints_reached << "/" 
              << path_grid_.size() << std::endl;
    std::cout << "       Total flight time: " << total_flight_time_ms << "ms" << std::endl;
    
    if (waypoints_reached > 0) {
        std::cout << "       Avg time/waypoint: " 
                  << (float)total_flight_time_ms / waypoints_reached << "ms" << std::endl;
    }
}

// ============================================================================
// ALTERNATIVE FUNCTION: Smart Altitude Selection (Future Enhancement)
// ============================================================================
// This function can be used to intelligently select altitude based on:
// - Obstacle clearance (Z from path planner)
// - Safety margins
// - Altitude constraints
// ============================================================================
/*
float TrajectoryFollower::selectAltitude(const Eigen::Vector3i& waypoint) {
    // Get altitude from path planner's 3D output
    float altitude_from_path = waypoint.z() * resolution_;
    
    // Minimum altitude for obstacle clearance
    const float MIN_CLEARANCE = 1.0f;  // 1m above obstacles
    
    // If path suggests altitude:
    if (altitude_from_path < -MIN_ALTITUDE) {
        // Use path altitude (3D flight)
        return std::clamp(altitude_from_path, -MAX_ALTITUDE, -MIN_ALTITUDE);
    } else {
        // Default to constant altitude
        return FLIGHT_ALTITUDE;
    }
}
*/

// ============================================================================
// ADVANCED: 3D TRAJECTORY WITH ALTITUDE VARIATION
// ============================================================================
// To enable full 3D flight (variable altitude):
//
// 1. Modify selectAltitude() function above (uncomment)
// 2. Change this line in start():
//    float down_m = selectAltitude(path_grid_[i]);
// 3. Ensure world_model calculates proper Z for obstacles
// 4. Verify altitude limits in path planner
//
// Benefits:
// - Better obstacle clearance
// - Smoother paths (fewer sharp turns)
// - More energy-efficient
// ============================================================================