#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <Eigen/Dense>
#include <memory> // Required for std::shared_ptr

class WorldModel {
public:
    WorldModel(double resolution);
    ~WorldModel();

    void buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // We keep the name "isCollision" to match your Path Planner
    bool isCollision(const Eigen::Vector3i& grid_pos) const;

    // New helper functions
    Eigen::Vector3i worldToGrid(const Eigen::Vector3d& ned_pos) const;
    Eigen::Vector3d gridToWorld(const Eigen::Vector3i& grid_pos) const;

private:
    // Changed to smart pointer for better memory management
    std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> octree;
    double resolution_;
};