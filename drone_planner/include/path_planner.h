#pragma once

// Forward declaration
class WorldModel;

#include <vector>
#include <queue>
#include <unordered_map>
#include <string>
#include <Eigen/Dense>

// Node struct
struct Node {
    Eigen::Vector3i pos;
    double g, h, f;
    Node* parent;

    Node(Eigen::Vector3i p) : pos(p), g(0), h(0), f(0), parent(nullptr) {}
};

// Function Declaration: pass world as const reference
std::vector<Eigen::Vector3i> A_star_search(
    const Eigen::Vector3i& start,
    const Eigen::Vector3i& goal,
    const WorldModel& world
);
