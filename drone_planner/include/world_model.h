#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <Eigen/Dense>
#include <memory>

class WorldModel {
public:
    WorldModel(double resolution);

    void buildMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool isCollisionFree(const Eigen::Vector3f& point) const;

    double getResolution() const { return resolution_; }
    Eigen::Vector3i worldToGrid(const Eigen::Vector3d& ned_pos) const;
    Eigen::Vector3d gridToWorld(const Eigen::Vector3i& grid_pos) const;

private:
    std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> octree;
    double resolution_;
};

#endif // WORLD_MODEL_H
