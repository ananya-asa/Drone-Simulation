#include "world_model.h"
#include <iostream>
#include <cmath>

WorldModel::WorldModel(double resolution) : resolution_(resolution) {
    octree = std::make_shared<pcl::octree::OctreePointCloud<pcl::PointXYZ>>(resolution);
    std::cout << "WorldModel created with " << resolution << "m resolution." << std::endl;
}

void WorldModel::buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "Building map with " << cloud->points.size() << " points..." << std::endl;

    // FIX: Make the world MASSIVE - 100m x 100m x 20m
    octree->defineBoundingBox(-50.0, -50.0, -5.0, 50.0, 50.0, 15.0);

    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
}

bool WorldModel::isCollisionFree(const Eigen::Vector3f& point) const {
    std::vector<int> indices;
    std::vector<float> distances;
    
    const int K = 1;
    if (octree->nearestKSearch(point, K, indices, distances) > 0) {
        // If nearest point is closer than 0.2m, it's a collision
        return distances[0] > (0.2 * 0.2);
    }
    return true;
}
