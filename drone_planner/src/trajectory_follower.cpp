#include "trajectory_follower.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::this_thread;
using mavsdk::Offboard;
using namespace std::chrono;

TrajectoryFollower::TrajectoryFollower(std::shared_ptr<mavsdk::Telemetry> telemetry,
                                       std::shared_ptr<mavsdk::Offboard> offboard,
                                       const std::vector<Eigen::Vector3i>& path,
                                       double resolution)
    : _telemetry(telemetry), _offboard(offboard), _path(path), _resolution(resolution)
{
    std::cout << "TrajectoryFollower created. Ready to follow " << _path.size() << " waypoints." << std::endl;
}

void TrajectoryFollower::start() {
    std::cout << "--- TrajectoryFollower: STARTING ---" << std::endl;

    for (size_t i = 1; i < _path.size(); ++i) {
        const auto& target_grid = _path[i];

        Offboard::PositionNedYaw target_pos{};
        target_pos.north_m = target_grid.x() * _resolution;
        target_pos.east_m = target_grid.y() * _resolution;
        target_pos.down_m = -2.5f;  // Maintain altitude
        target_pos.yaw_deg = 0.0f;

        std::cout << "Flying to waypoint " << i << ": ("
                  << target_pos.north_m << ", "
                  << target_pos.east_m << ", "
                  << target_pos.down_m << ")" << std::endl;

        _offboard->set_position_ned(target_pos);

        double distance_to_target = 999.0;
        do {
            distance_to_target = get_distance_to_target(target_grid);
            sleep_for(milliseconds(100));
        } while (distance_to_target > 0.1);  // <--- Fixed: Threshold smaller than waypoint distance

        std::cout << "  ... Arrived at waypoint " << i << std::endl;
    }

    std::cout << "--- TrajectoryFollower: FINISHED ---" << std::endl;
}

double TrajectoryFollower::get_distance_to_target(const Eigen::Vector3i& target_grid) {
    auto drone_pos_ned = _telemetry->position_velocity_ned().position;

    double target_x = target_grid.x() * _resolution;
    double target_y = target_grid.y() * _resolution;

    double dx = target_x - drone_pos_ned.north_m;
    double dy = target_y - drone_pos_ned.east_m;

    return std::sqrt(dx*dx + dy*dy);
}
