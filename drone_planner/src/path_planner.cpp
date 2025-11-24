#include "path_planner.h"

double heuristic(const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    return (a - b).cast<double>().norm();
}

std::vector<Eigen::Vector3i> A_star_search(
    const Eigen::Vector3i& start,
    const Eigen::Vector3i& goal,
    const WorldModel& world) {
    
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<Eigen::Vector3i, double, Vector3iHash> g_costs;
    std::unordered_map<Eigen::Vector3i, Eigen::Vector3i, Vector3iHash> came_from;
    std::unordered_set<Eigen::Vector3i, Vector3iHash> closed_set;

    Node start_node;
    start_node.position = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = heuristic(start, goal);
    start_node.parent = start;

    open_set.push(start_node);
    g_costs[start] = 0.0;

    std::vector<Eigen::Vector3i> directions = {
        Eigen::Vector3i(1, 0, 0),   Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0),   Eigen::Vector3i(0, -1, 0),
        Eigen::Vector3i(0, 0, 1),   Eigen::Vector3i(0, 0, -1),
        Eigen::Vector3i(1, 1, 0),   Eigen::Vector3i(1, -1, 0),
        Eigen::Vector3i(-1, 1, 0),  Eigen::Vector3i(-1, -1, 0),
        Eigen::Vector3i(1, 0, 1),   Eigen::Vector3i(1, 0, -1),
        Eigen::Vector3i(-1, 0, 1),  Eigen::Vector3i(-1, 0, -1),
        Eigen::Vector3i(0, 1, 1),   Eigen::Vector3i(0, 1, -1),
        Eigen::Vector3i(0, -1, 1),  Eigen::Vector3i(0, -1, -1),
        Eigen::Vector3i(1, 1, 1),   Eigen::Vector3i(1, 1, -1),
        Eigen::Vector3i(1, -1, 1),  Eigen::Vector3i(1, -1, -1),
        Eigen::Vector3i(-1, 1, 1),  Eigen::Vector3i(-1, 1, -1),
        Eigen::Vector3i(-1, -1, 1), Eigen::Vector3i(-1, -1, -1)
    };

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        if (closed_set.find(current.position) != closed_set.end()) {
            continue;
        }

        closed_set.insert(current.position);

        if (current.position == goal) {
            std::vector<Eigen::Vector3i> path;
            Eigen::Vector3i pos = goal;
            while (pos != start) {
                path.push_back(pos);
                pos = came_from[pos];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& dir : directions) {
            Eigen::Vector3i neighbor = current.position + dir;

            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            Eigen::Vector3f neighbor_pos = neighbor.cast<float>() * world.getResolution();
            
            std::cout << "[DEBUG] Checking grid " << neighbor.transpose() 
                      << " -> world " << neighbor_pos.transpose();

            if (!world.isCollisionFree(neighbor_pos)) {
                std::cout << " -> COLLISION" << std::endl;
                continue;
            }
            std::cout << " -> free" << std::endl;

            double tentative_g = g_costs[current.position] + dir.cast<double>().norm();

            if (g_costs.find(neighbor) == g_costs.end() || tentative_g < g_costs[neighbor]) {
                g_costs[neighbor] = tentative_g;
                came_from[neighbor] = current.position;

                Node neighbor_node;
                neighbor_node.position = neighbor;
                neighbor_node.g_cost = tentative_g;
                neighbor_node.h_cost = heuristic(neighbor, goal);
                neighbor_node.parent = current.position;

                open_set.push(neighbor_node);
            }
        }
    }

    return std::vector<Eigen::Vector3i>();
}
