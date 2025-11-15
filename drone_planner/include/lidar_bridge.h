#ifndef LIDAR_BRIDGE_H
#define LIDAR_BRIDGE_H

#include <memory>
#include <mutex>
#include <atomic>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LIDARBridge {
public:
    LIDARBridge();
    ~LIDARBridge();

    bool subscribe();
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud();
    bool has_new_data();
    void reset_data_flag();
    int get_callback_count() const { return callback_count_; }

private:
    void lidar_callback(const gz::msgs::PointCloudPacked& msg);

    gz::transport::Node gz_node_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::mutex cloud_mutex_;
    std::atomic<bool> new_data_available_;
    int callback_count_;
};

#endif // LIDAR_BRIDGE_H
