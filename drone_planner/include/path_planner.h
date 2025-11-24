#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include "world_model.h"

struct Node {
    Eigen::Vector3i position;
    double g_cost;
    double h_cost;
    Eigen::Vector3i parent;

    double f_cost() const { return g_cost + h_cost; }

    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
};

struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& v) const {
        std::size_t h1 = std::hash<int>()(v.x());
        std::size_t h2 = std::hash<int>()(v.y());
        std::size_t h3 = std::hash<int>()(v.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

std::vector<Eigen::Vector3i> A_star_search(
    const Eigen::Vector3i& start,
    const Eigen::Vector3i& goal,
    const WorldModel& world
);

#endif // PATH_PLANNER_H
