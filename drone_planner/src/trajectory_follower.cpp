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
        float north_m = path_grid_[i].x() * resolution_;
        float east_m = path_grid_[i].y() * resolution_;
        float down_m = -2.5f;

        std::cout << "\nWaypoint " << i+1 << "/" << path_grid_.size() 
                  << ": Grid(" << path_grid_[i].x() << "," << path_grid_[i].y() << "," << path_grid_[i].z() << ")"
                  << " -> NED(" << north_m << "," << east_m << "," << down_m << ")" << std::endl;

        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = north_m;
        setpoint.east_m = east_m;
        setpoint.down_m = down_m;
        setpoint.yaw_deg = 0.0f;
        
        offboard_->set_position_ned(setpoint);
        sleep_for(milliseconds(200));

        bool reached = false;
        int timeout_count = 0;
        const int max_timeout = 100;
        const float distance_threshold = 0.15f;

        std::cout << "  Waiting to reach waypoint...";

        while (!reached && timeout_count < max_timeout) {
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
            
            if (timeout_count % 10 == 0) {
                std::cout << ".";
            }
        }

        if (!reached) {
            std::cout << " ✗ TIMEOUT (moved on after " << timeout_count << " iterations)" << std::endl;
        }

        sleep_for(milliseconds(500));
    }

    std::cout << "\nTrajectoryFollower: Path execution complete!" << std::endl;
}
