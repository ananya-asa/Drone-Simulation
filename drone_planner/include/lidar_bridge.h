#ifndef LIDAR_BRIDGE_H
#define LIDAR_BRIDGE_H

#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>
#include <atomic>

class LIDARBridge {
public:
    LIDARBridge();
    ~LIDARBridge();

    bool subscribe();
    bool has_new_data() const;
    void reset_data_flag();
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud();

private:
    void lidar_callback(const gz::msgs::LaserScan& msg);

    gz::transport::Node node_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::mutex cloud_mutex_;
    std::atomic<bool> new_data_flag_;
    std::atomic<int> callback_count_;
};

#endif // LIDAR_BRIDGE_H
