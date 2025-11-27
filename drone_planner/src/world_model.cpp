#include "world_model.h"
#include <iostream>
#include <cmath>
#include <cfloat> // For FLT_MAX
#include <vector>

WorldModel::WorldModel(double resolution) : resolution_(resolution) {
    octree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(resolution_);
    std::cout << "WorldModel created with " << resolution_ << "m resolution." << std::endl;
}

WorldModel::~WorldModel() {
    // Shared pointer handles deletion automatically
}

void WorldModel::buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // FIX 1: Don't print every frame to keep logs clean
    // std::cout << "Building map with " << cloud->points.size() << " points..." << std::endl;
    
    // FIX 2: Clear old octree
    octree->deleteTree(); 
    // We don't need to re-allocate the pointer, just clear the tree structure
    
    // FIX 3: Dynamic Bounding Box
    float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
    float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
    
    if (cloud->points.empty()) {
        // Safety fallback if cloud is empty
        min_x = -10; min_y = -10; min_z = -5;
        max_x = 10; max_y = 10; max_z = 5;
    } else {
        for (const auto& pt : cloud->points) {
            if (pt.x < min_x) min_x = pt.x;
            if (pt.y < min_y) min_y = pt.y;
            if (pt.z < min_z) min_z = pt.z;
            if (pt.x > max_x) max_x = pt.x;
            if (pt.y > max_y) max_y = pt.y;
            if (pt.z > max_z) max_z = pt.z;
        }
        // Add safety margins (2 meters buffer)
        float margin = 2.0f;
        min_x -= margin; min_y -= margin; min_z -= margin;
        max_x += margin; max_y += margin; max_z += margin;
    }
    
    octree->defineBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
}

bool WorldModel::isCollision(const Eigen::Vector3i& grid_pos) const {
    // Convert grid to world coordinate
    pcl::PointXYZ search_point(
        grid_pos.x() * resolution_, 
        grid_pos.y() * resolution_, 
        grid_pos.z() * resolution_
    );
    
    // SAFETY MARGIN: Drone radius + Error margin
    // 0.8m is a good "Paranoid" safety distance
    const float SAFETY_MARGIN = 0.8f; 
    const float SAFETY_RADIUS_SQ = SAFETY_MARGIN * SAFETY_MARGIN;
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    // Check 1 nearest neighbor. If the CLOSEST point is far away, we are safe.
    int k_neighbors = 1; 
    int result = octree->nearestKSearch(search_point, k_neighbors, indices, distances);
    
    if (result == 0) {
        return false; // No points found, SAFE (No Collision)
    }
    
    // If the closest point is closer than our safety margin, it's a collision
    if (distances[0] < SAFETY_RADIUS_SQ) {
        return true; // COLLISION!
    }
    
    return false; // Safe
}

// Helper functions
Eigen::Vector3i WorldModel::worldToGrid(const Eigen::Vector3d& ned_pos) const {
    return Eigen::Vector3i(
        std::round(ned_pos.x() / resolution_), 
        std::round(ned_pos.y() / resolution_), 
        std::round(ned_pos.z() / resolution_) 
    );
}

Eigen::Vector3d WorldModel::gridToWorld(const Eigen::Vector3i& grid_pos) const {
    return Eigen::Vector3d(
        grid_pos.x() * resolution_,
        grid_pos.y() * resolution_,
        grid_pos.z() * resolution_
    );
}