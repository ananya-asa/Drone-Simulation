#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

#include <Eigen/Dense>

class WorldModel {
public:
    WorldModel(double resolution);
    ~WorldModel();

    void buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    bool isCollision(const Eigen::Vector3i& grid_pos) const;

private:
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree;
    double _resolution;
};
