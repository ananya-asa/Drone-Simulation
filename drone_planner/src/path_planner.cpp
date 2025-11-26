#include "path_planner.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <iostream>
#include <algorithm>

double heuristic(const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    // Euclidean distance heuristic
    double dx = a.x() - b.x();
    double dy = a.y() - b.y();
    double dz = a.z() - b.z();
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<Eigen::Vector3i> A_star_search(
    const Eigen::Vector3i& start,
    const Eigen::Vector3i& goal,
    const WorldModel& world) {
    
    std::cout << "\n[PLANNER] Starting A* from " << start.transpose() 
              << " to " << goal.transpose() << std::endl;
    
    // Check if start or goal are in collision
    Eigen::Vector3f start_world = start.cast<float>() * world.getResolution();
    Eigen::Vector3f goal_world = goal.cast<float>() * world.getResolution();
    
    if (!world.isCollisionFree(start_world)) {
        std::cerr << "[PLANNER] ERROR: Start position is in collision!" << std::endl;
        return std::vector<Eigen::Vector3i>();
    }
    
    if (!world.isCollisionFree(goal_world)) {
        std::cerr << "[PLANNER] ERROR: Goal position is in collision!" << std::endl;
        return std::vector<Eigen::Vector3i>();
    }
    
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<int, double> g_costs;
    std::unordered_map<int, Eigen::Vector3i> came_from;
    std::unordered_set<int> closed_set;
    
    auto hash_pos = [](const Eigen::Vector3i& p) {
        return ((p.x() + 500) * 73856093) ^ ((p.y() + 500) * 19349663) ^ ((p.z() + 500) * 83492791);
    };
    
    Node start_node;
    start_node.position = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = heuristic(start, goal);
    start_node.parent = start;
    
    open_set.push(start_node);
    g_costs[hash_pos(start)] = 0.0;
    
    // 26-directional movement (including diagonals)
    std::vector<Eigen::Vector3i> directions = {
        // Cardinal directions
        Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
        Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1),
        
        // Diagonal XY
        Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, -1, 0),
        Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, -1, 0),
        
        // Diagonal XZ
        Eigen::Vector3i(1, 0, 1), Eigen::Vector3i(1, 0, -1),
        Eigen::Vector3i(-1, 0, 1), Eigen::Vector3i(-1, 0, -1),
        
        // Diagonal YZ
        Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(0, 1, -1),
        Eigen::Vector3i(0, -1, 1), Eigen::Vector3i(0, -1, -1),
        
        // 3D diagonals
        Eigen::Vector3i(1, 1, 1), Eigen::Vector3i(1, 1, -1),
        Eigen::Vector3i(1, -1, 1), Eigen::Vector3i(1, -1, -1),
        Eigen::Vector3i(-1, 1, 1), Eigen::Vector3i(-1, 1, -1),
        Eigen::Vector3i(-1, -1, 1), Eigen::Vector3i(-1, -1, -1)
    };
    
    int iterations = 0;
    const int MAX_ITERATIONS = 100000;
    int nodes_expanded = 0;
    
    while (!open_set.empty() && iterations < MAX_ITERATIONS) {
        iterations++;
        
        Node current = open_set.top();
        open_set.pop();
        
        int curr_hash = hash_pos(current.position);
        if (closed_set.find(curr_hash) != closed_set.end()) {
            continue;
        }
        
        closed_set.insert(curr_hash);
        nodes_expanded++;
        
        // GOAL REACHED
        if (current.position == goal) {
            std::cout << "[PLANNER] ✓ Path found in " << iterations << " iterations! ("
                      << nodes_expanded << " nodes expanded)" << std::endl;
            
            std::vector<Eigen::Vector3i> path;
            Eigen::Vector3i pos = goal;
            
            while (pos != start) {
                path.push_back(pos);
                int pos_hash = hash_pos(pos);
                if (came_from.find(pos_hash) == came_from.end()) {
                    std::cerr << "[PLANNER] ERROR: Path reconstruction failed!" << std::endl;
                    return std::vector<Eigen::Vector3i>();
                }
                pos = came_from[pos_hash];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            
            std::cout << "[PLANNER] Path length: " << path.size() << " waypoints" << std::endl;
            return path;
        }
        
        // EXPLORE NEIGHBORS
        for (const auto& dir : directions) {
            Eigen::Vector3i neighbor = current.position + dir;
            int neighbor_hash = hash_pos(neighbor);
            
            if (closed_set.find(neighbor_hash) != closed_set.end()) {
                continue;
            }
            
            // Convert to world and check collision
            Eigen::Vector3f neighbor_world = neighbor.cast<float>() * world.getResolution();
            
            if (!world.isCollisionFree(neighbor_world)) {
                continue;  // Skip this neighbor - it's in collision
            }
            
            // Calculate costs
            double move_cost = dir.cast<double>().norm();
            double tentative_g = g_costs[curr_hash] + move_cost;
            
            // Only add if this is a better path
            if (g_costs.find(neighbor_hash) == g_costs.end() || tentative_g < g_costs[neighbor_hash]) {
                g_costs[neighbor_hash] = tentative_g;
                came_from[neighbor_hash] = current.position;
                
                Node neighbor_node;
                neighbor_node.position = neighbor;
                neighbor_node.g_cost = tentative_g;
                neighbor_node.h_cost = heuristic(neighbor, goal);
                neighbor_node.parent = current.position;
                
                open_set.push(neighbor_node);
            }
        }
    }
    
    std::cerr << "[PLANNER] ✗ NO PATH FOUND after " << iterations << " iterations ("
              << nodes_expanded << " nodes expanded)" << std::endl;
    std::cerr << "[PLANNER] Debug: Start=" << start.transpose() << ", Goal=" << goal.transpose() << std::endl;
    return std::vector<Eigen::Vector3i>();
}