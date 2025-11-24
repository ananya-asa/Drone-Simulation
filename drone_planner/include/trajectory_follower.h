#ifndef TRAJECTORY_FOLLOWER_H
#define TRAJECTORY_FOLLOWER_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <vector>
#include <Eigen/Dense>

class TrajectoryFollower {
public:
    TrajectoryFollower(
        std::shared_ptr<mavsdk::Telemetry> telemetry,
        std::shared_ptr<mavsdk::Offboard> offboard,
        const std::vector<Eigen::Vector3i>& path_grid,
        double resolution);

    void start();

private:
    std::shared_ptr<mavsdk::Telemetry> telemetry_;
    std::shared_ptr<mavsdk::Offboard> offboard_;
    std::vector<Eigen::Vector3i> path_grid_;
    double resolution_;
};

#endif // TRAJECTORY_FOLLOWER_H
