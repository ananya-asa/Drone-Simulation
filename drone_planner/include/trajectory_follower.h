#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

class TrajectoryFollower {
public:
    TrajectoryFollower(std::shared_ptr<mavsdk::Telemetry> telemetry,
                       std::shared_ptr<mavsdk::Offboard> offboard,
                       const std::vector<Eigen::Vector3i>& path,
                       double resolution);

    void start();

private:
    std::shared_ptr<mavsdk::Telemetry> _telemetry;
    std::shared_ptr<mavsdk::Offboard> _offboard;
    std::vector<Eigen::Vector3i> _path;
    double _resolution;

    double get_distance_to_target(const Eigen::Vector3i& target_grid);
};
