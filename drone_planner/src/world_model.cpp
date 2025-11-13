#include "world_model.h"
#include <iostream>

WorldModel::WorldModel(double resolution)
    : _resolution(resolution),
      octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution))
{
    std::cout << "WorldModel created with " << _resolution << "m resolution." << std::endl;
}

WorldModel::~WorldModel() {
    delete octree;
}

void WorldModel::buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "Building map with " << cloud->points.size() << " points..." << std::endl;
    octree->defineBoundingBox(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
    std::cout << "Map built." << std::endl;
}

bool WorldModel::isCollision(const Eigen::Vector3i& grid_pos) const { 
    pcl::PointXYZ searchPoint;
    searchPoint.x = grid_pos.x() * this->_resolution; 
    searchPoint.y = grid_pos.y() * this->_resolution; 
    searchPoint.z = grid_pos.z() * this->_resolution;
    
    // Use radius search instead of voxel search for better detection
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float search_radius = _resolution * 1.5f; // Search slightly larger than voxel
    
    int found = octree->radiusSearch(searchPoint, search_radius, 
                                      pointIdxRadiusSearch, 
                                      pointRadiusSquaredDistance);
    
    bool collision = (found > 0);
    
    // Debug output for key positions
    if (grid_pos.x() >= 4 && grid_pos.x() <= 6 && grid_pos.y() >= -1 && grid_pos.y() <= 1) {
        std::cout << "[DEBUG] Checking grid (" << grid_pos.x() << "," << grid_pos.y() << "," << grid_pos.z() 
                  << ") -> world (" << searchPoint.x << "," << searchPoint.y << "," << searchPoint.z 
                  << ") -> " << (collision ? "COLLISION" : "free") << " (found " << found << " points)" << std::endl;
    }
    
    return collision;
}
