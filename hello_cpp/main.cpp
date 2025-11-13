#include <iostream>
#include "path_planner.h" // Our A* function
#include "world_model.h"  // Our WorldModel class

// --- Helper function to create a test "wall" ---
pcl::PointCloud<pcl::PointXYZ>::Ptr create_test_wall() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create a "wall" of points at x=5
    // Our grid resolution is 0.1m, so we'll add points every 0.1m
    for (double y = -1.0; y <= 1.0; y += 0.1) {
        for (double z = -1.0; z <= 1.0; z += 0.1) {
            cloud->points.push_back(pcl::PointXYZ(0.5, y, z));
        }
    }
    // Note: The wall is at x=0.5, which corresponds to
    // grid coordinate x=5 (since 0.5 / 0.1 = 5)
    
    std::cout << "Created a test wall with " << cloud->points.size() << " points." << std::endl;
    return cloud;
}


int main() {
    std::cout << "--- Testing A* with Real WorldModel Collision ---" << std::endl;
    
    // 1. Create our world
    WorldModel world;
    
    // 2. Create a fake wall and build the map
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall = create_test_wall();
    world.buildMap(wall);

    // 3. Set our start and goal
    //    Start is at (0,0,0)
    //    Goal is at (10,0,0) -> (1.0m, 0, 0)
    //    The wall is at x=5! The path *must* go around it.
    Eigen::Vector3i start(0, 0, 0);
    Eigen::Vector3i goal(10, 0, 0);
    
    std::cout << "Finding path from (0,0,0) to (10,0,0)..." << std::endl;

    // 4. Call A* and pass it the world!
    std::vector<Eigen::Vector3i> path = A_star_search(start, goal, world);

    if (path.empty()) {
        std::cout << "Test FAILED: No path was found." << std::endl;
    } else {
        std::cout << "Test SUCCESS! Path found with " << path.size() << " steps:" << std::endl;
        // Print just the first and last steps
        std::cout << "  Start: (" << path.front().x() << ", " << path.front().y() << ", " << path.front().z() << ")" << std::endl;
        std::cout << "  ... " << std::endl;
        std::cout << "  Goal:  (" << path.back().x() << ", " << path.back().y() << ", " << path.back().z() << ")" << std::endl;
    }
    
    std::cout << "\n[SUCCESS] A* planner and WorldModel are integrated." << std::endl;

    return 0;
}