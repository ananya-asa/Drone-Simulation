#include "lidar_bridge.h"
#include <iostream>
#include <cstring>

LIDARBridge::LIDARBridge() 
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>),
      new_data_available_(false),
      callback_count_(0) {}

LIDARBridge::~LIDARBridge() {}

bool LIDARBridge::subscribe() {
    if (!gz_node_.Subscribe("/lidar", &LIDARBridge::lidar_callback, this)) {
        std::cerr << "[LIDAR_BRIDGE] Failed to subscribe to /lidar topic" << std::endl;
        return false;
    }
    std::cout << "[LIDAR_BRIDGE] Successfully subscribed to /lidar topic" << std::endl;
    return true;
}

void LIDARBridge::lidar_callback(const gz::msgs::PointCloudPacked& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    uint32_t num_points = msg.width() * msg.height();
    temp_cloud->width = num_points;
    temp_cloud->height = 1;
    temp_cloud->is_dense = false;
    temp_cloud->points.resize(num_points);

    // Parse packed point cloud data
    int point_step = msg.point_step();
    for (uint32_t i = 0; i < num_points; ++i) {
        memcpy(&temp_cloud->points[i].x, &msg.data()[i * point_step], sizeof(float));
        memcpy(&temp_cloud->points[i].y, &msg.data()[i * point_step + sizeof(float)], sizeof(float));
        memcpy(&temp_cloud->points[i].z, &msg.data()[i * point_step + 2 * sizeof(float)], sizeof(float));
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_ = temp_cloud;
        callback_count_++;
    }

    new_data_available_ = true;

    if (callback_count_ % 20 == 0) {
        std::cout << "[LIDAR_BRIDGE] Received " << num_points << " points (callback #" 
                  << callback_count_ << ")" << std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LIDARBridge::get_cloud() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return cloud_;
}

bool LIDARBridge::has_new_data() {
    return new_data_available_;
}

void LIDARBridge::reset_data_flag() {
    new_data_available_ = false;
}
