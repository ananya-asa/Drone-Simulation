#include "lidar_bridge.h"
#include <iostream>

LIDARBridge::LIDARBridge() 
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>),
      new_data_flag_(false),
      callback_count_(0) {
}

LIDARBridge::~LIDARBridge() {
    std::cout << "[LIDAR_BRIDGE] Shutting down. Total callbacks: " 
              << callback_count_.load() << std::endl;
}

bool LIDARBridge::subscribe() {
    std::string topic = "/lidar";
    
    if (!node_.Subscribe(topic, &LIDARBridge::lidar_callback, this)) {
        std::cerr << "[LIDAR_BRIDGE] Failed to subscribe to " << topic << std::endl;
        return false;
    }
    
    std::cout << "[LIDAR_BRIDGE] Successfully subscribed to " << topic << " topic" << std::endl;
    return true;
}

void LIDARBridge::lidar_callback(const gz::msgs::LaserScan& msg) {
    callback_count_++;
    
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_->clear();
    
    int num_ranges = msg.ranges_size();
    double angle = msg.angle_min();
    double angle_step = msg.angle_step();
    
    for (int i = 0; i < num_ranges; ++i) {
        double range = msg.ranges(i);
        
        if (range >= msg.range_min() && range <= msg.range_max()) {
            float x = range * cos(angle);
            float y = range * sin(angle);
            float z = 0.0f;
            
            cloud_->points.push_back(pcl::PointXYZ(x, y, z));
        }
        
        angle += angle_step;
    }
    
    new_data_flag_ = true;
    
    if (callback_count_ % 10 == 0) {
        std::cout << "[LIDAR] Data received! Building map with " 
                  << cloud_->points.size() << " points" << std::endl;
    }
}

bool LIDARBridge::has_new_data() const {
    return new_data_flag_.load();
}

void LIDARBridge::reset_data_flag() {
    new_data_flag_ = false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LIDARBridge::get_cloud() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
    return cloud_copy;
}
