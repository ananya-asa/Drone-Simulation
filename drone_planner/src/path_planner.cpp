#include "path_planner.h"
#include "world_model.h"
#include <iostream>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

// --- HELPER STRUCTS (Internal use only) ---

// Hash function for Eigen::Vector3i
struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& v) const {
        std::size_t seed = 0;
        std::hash<int> hasher;
        seed ^= hasher(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hasher(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hasher(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

// Equality function
struct Vector3iEqual {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

// Comparator for priority queue
struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f > b->f; // Smallest f is top priority
    }
};

// --- HELPER FUNCTIONS ---

double calculate_heuristic(const Eigen::Vector3i& pos, const Eigen::Vector3i& goal) {
    return std::sqrt(std::pow(pos.x() - goal.x(), 2) + 
                     std::pow(pos.y() - goal.y(), 2) + 
                     std::pow(pos.z() - goal.z(), 2));
}

std::vector<Eigen::Vector3i> reconstruct_path(Node* goal_node) {
    std::vector<Eigen::Vector3i> path;
    Node* current = goal_node;
    while (current) {
        path.push_back(current->pos);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::Vector3i> get_neighbors(const Eigen::Vector3i& pos) {
    std::vector<Eigen::Vector3i> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                neighbors.emplace_back(pos.x() + dx, pos.y() + dy, pos.z() + dz);
            }
        }
    }
    return neighbors;
}

// --- MAIN ALGORITHM ---

std::vector<Eigen::Vector3i> A_star_search(
    const Eigen::Vector3i& start,
    const Eigen::Vector3i& goal,
    const WorldModel& world
) {
    // Safety check: Don't start if we are inside a wall!
    if (world.isCollision(start)) {
        std::cout << "[PLANNER] ERROR: Start position is in collision!" << std::endl;
        return {}; 
    }
    
    // Setup lists
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_set;
    std::unordered_map<Eigen::Vector3i, Node*, Vector3iHash, Vector3iEqual> all_nodes;

    // Setup start node
    Node* start_node = new Node(start);
    start_node->g = 0;
    start_node->h = calculate_heuristic(start, goal);
    start_node->f = start_node->g + start_node->h;
    
    open_set.push(start_node);
    all_nodes[start] = start_node;
    
    int iterations = 0;

    while (!open_set.empty()) {
        iterations++;
        Node* current = open_set.top();
        open_set.pop(); 

        // Check Goal
        if (current->pos == goal) {
            std::cout << "[PLANNER] ✓ Path found in " << iterations << " iterations!" << std::endl;
            auto path = reconstruct_path(current);
            // Cleanup
            for (auto& pair : all_nodes) delete pair.second;
            return path;
        }

        // Check Neighbors
        for (const auto& neighbor_pos : get_neighbors(current->pos)) {
            
            if (world.isCollision(neighbor_pos)) continue;

            double tentative_g = current->g + (neighbor_pos - current->pos).cast<float>().norm();

            if (all_nodes.find(neighbor_pos) == all_nodes.end()) {
                // New Node
                Node* neighbor_node = new Node(neighbor_pos);
                neighbor_node->parent = current;
                neighbor_node->g = tentative_g;
                neighbor_node->h = calculate_heuristic(neighbor_pos, goal);
                neighbor_node->f = neighbor_node->g + neighbor_node->h;
                
                open_set.push(neighbor_node);
                all_nodes[neighbor_pos] = neighbor_node;
            } else {
                // Existing Node
                Node* neighbor_node = all_nodes[neighbor_pos];
                if (tentative_g < neighbor_node->g) {
                    neighbor_node->parent = current;
                    neighbor_node->g = tentative_g;
                    neighbor_node->f = neighbor_node->g + neighbor_node->h;
                    open_set.push(neighbor_node);
                }
            }
        }
    }
    
    // Cleanup if fail
    for (auto& pair : all_nodes) delete pair.second;
    return {};
}