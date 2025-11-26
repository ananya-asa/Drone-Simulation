#include "world_model.h"
#include <iostream>
#include <cmath>

WorldModel::WorldModel(double resolution) : resolution_(resolution) {
    octree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(resolution);
    std::cout << "WorldModel created with " << resolution << "m resolution." << std::endl;
}

void WorldModel::buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "Building map with " << cloud->points.size() << " points..." << std::endl;
    octree->defineBoundingBox(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
}

bool WorldModel::isCollisionFree(const Eigen::Vector3f& point) const {
    pcl::PointXYZ search_point(point.x(), point.y(), point.z());
    std::vector<int> indices;
    std::vector<float> distances;

    if (octree->nearestKSearch(search_point, 1, indices, distances) > 0) {
        int collision_count = 0;
        for (float dist : distances) {
            if (dist < 0.2 * 0.2) {
                collision_count++;
            }
        }
        std::cout << " (found " << collision_count << " points)";
        return collision_count == 0;
    }

    std::cout << " (found 0 points)";
    return true;
}

// ALTITUDE-CONVERSION FIX
Eigen::Vector3i WorldModel::worldToGrid(const Eigen::Vector3d& ned_pos) const {
    return Eigen::Vector3i(
        std::round(ned_pos.x() / resolution_),   // North
        std::round(ned_pos.y() / resolution_),   // East
        std::round(-ned_pos.z() / resolution_)   // FIX: Negate Down for altitude!
    );
}
Eigen::Vector3d WorldModel::gridToWorld(const Eigen::Vector3i& grid_pos) const {
    return Eigen::Vector3d(
        grid_pos.x() * resolution_,
        grid_pos.y() * resolution_,
        -grid_pos.z() * resolution_
    );
}
