#include "trajectory_follower.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

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

    for (size_t i = 0; i < path_grid_.size(); ++i) {
        // Convert grid coordinates to world coordinates (NED frame)
        float north_m = path_grid_[i].x() * resolution_;
        float east_m = path_grid_[i].y() * resolution_;
        float down_m = -2.5f; // Fixed altitude

        std::cout << "Waypoint " << i+1 << "/" << path_grid_.size() 
                  << ": Grid(" << path_grid_[i].x() << "," << path_grid_[i].y() << "," << path_grid_[i].z() << ")"
                  << " -> NED(" << north_m << "," << east_m << "," << down_m << ")" << std::endl;

        // Send setpoint
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = north_m;
        setpoint.east_m = east_m;
        setpoint.down_m = down_m;
        setpoint.yaw_deg = 0.0f;
        
        offboard_->set_position_ned(setpoint);

        // Wait for drone to reach waypoint
        bool reached = false;
        int timeout_count = 0;
        const int max_timeout = 50; // 5 seconds

        while (!reached && timeout_count < max_timeout) {
            Telemetry::PositionNed current_pos = telemetry_->position_velocity_ned().position;
            
            float distance = std::sqrt(
                std::pow(current_pos.north_m - north_m, 2) +
                std::pow(current_pos.east_m - east_m, 2) +
                std::pow(current_pos.down_m - down_m, 2)
            );

            if (distance < 0.3) { // 30cm threshold
                reached = true;
                std::cout << "  -> Reached waypoint " << i+1 << std::endl;
            }

            sleep_for(milliseconds(100));
            timeout_count++;
        }

        if (!reached) {
            std::cout << "  -> Timeout at waypoint " << i+1 << ", moving to next" << std::endl;
        }
    }

    std::cout << "TrajectoryFollower: Path execution complete!" << std::endl;
}
