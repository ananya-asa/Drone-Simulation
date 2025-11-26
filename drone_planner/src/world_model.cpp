#include "world_model.h"
#include <pcl/octree/octree_search.h>
#include <cmath>
#include <cfloat>  // FIX: Add this for FLT_MAX

WorldModel::WorldModel(double resolution) : resolution_(resolution) {
    octree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(resolution);
    std::cout << "WorldModel created with " << resolution << "m resolution." << std::endl;
}

void WorldModel::buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "Building map with " << cloud->points.size() << " points..." << std::endl;
    
    // FIX 1: CLEAR OLD OCTREE BEFORE REBUILDING
    octree.reset();
    octree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(0.2);
    
    // FIX 2: DYNAMIC BOUNDING BOX - Auto-compute from cloud, not hardcoded
    float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
    float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
    
    for (const auto& pt : cloud->points) {
        min_x = std::min(min_x, pt.x);
        min_y = std::min(min_y, pt.y);
        min_z = std::min(min_z, pt.z);
        max_x = std::max(max_x, pt.x);
        max_y = std::max(max_y, pt.y);
        max_z = std::max(max_z, pt.z);
    }
    
    // Add safety margins
    float margin = 0.5f;
    min_x -= margin; min_y -= margin; min_z -= margin;
    max_x += margin; max_y += margin; max_z += margin;
    
    std::cout << "[OCTREE] Bounding box: [" << min_x << ", " << max_x << "] x ["
              << min_y << ", " << max_y << "] x [" << min_z << ", " << max_z << "]" << std::endl;
    
    octree->defineBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
}

// FIX 3: IMPROVED COLLISION DETECTION WITH SAFETY MARGIN
bool WorldModel::isCollisionFree(const Eigen::Vector3f& point) const {
    pcl::PointXYZ search_point(point.x(), point.y(), point.z());
    
    // SAFETY MARGIN: Drone radius is ~0.33m, we check 0.4m buffer
    const float SAFETY_MARGIN = 0.7f;
    const float SAFETY_RADIUS_SQ = SAFETY_MARGIN * SAFETY_MARGIN;
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    // FIX 4: Check MULTIPLE nearby points, not just 1
    int k_neighbors = 5;  // Check 5 nearest neighbors instead of 1
    int result = octree->nearestKSearch(search_point, k_neighbors, indices, distances);
    
    if (result == 0) {
        // No points found near this location - it's DEFINITELY free
        return true;
    }
    
    // Check if any neighbor is within safety radius
    for (float dist : distances) {
        if (dist < SAFETY_RADIUS_SQ) {
            // Too close to an obstacle
            return false;
        }
    }
    
    // All neighbors are far enough away - path is clear
    return true;
}

// COORDINATE CONVERSION: NED (North-East-Down) <-> Grid
Eigen::Vector3i WorldModel::worldToGrid(const Eigen::Vector3d& ned_pos) const {
    return Eigen::Vector3i(
        std::round(ned_pos.x() / resolution_),   // North -> X
        std::round(ned_pos.y() / resolution_),   // East  -> Y
        std::round(-ned_pos.z() / resolution_)   // Down (negate for altitude)
    );
}

Eigen::Vector3d WorldModel::gridToWorld(const Eigen::Vector3i& grid_pos) const {
    return Eigen::Vector3d(
        grid_pos.x() * resolution_,
        grid_pos.y() * resolution_,
        -grid_pos.z() * resolution_
    );
}