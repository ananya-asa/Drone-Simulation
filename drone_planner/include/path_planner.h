#pragma once

#include <vector>
#include <Eigen/Dense>

// Forward declaration
class WorldModel;

// The Node struct MUST be defined here so everyone can see it
struct Node {
    Eigen::Vector3i pos;
    double g, h, f;
    Node* parent;

    // Constructor
    Node(Eigen::Vector3i p) : pos(p), g(0), h(0), f(0), parent(nullptr) {}
};

// Main Function Declaration
std::vector<Eigen::Vector3i> A_star_search(
    const Eigen::Vector3i& start,
    const Eigen::Vector3i& goal,
    const WorldModel& world
);