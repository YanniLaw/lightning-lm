#ifndef LIGHTNING_CORE_LIO_POINTCLOUD_PREPROCESS_H_
#define LIGHTNING_CORE_LIO_POINTCLOUD_PREPROCESS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "common/sensor_data.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

// 预处理程序：统一各雷达的点格式和逐点时间。
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    bool Init(const YAML::Node &yaml);
    /// processors
    bool Process(const sensor_msgs::msg::PointCloud2 &msg, TimedPointCloud &output);
    bool Process(const livox_ros_driver2::msg::CustomMsg &msg, TimedPointCloud &output);

   private:
    void HandleLivox(const livox_ros_driver2::msg::CustomMsg &msg);
    void Oust64Handler(const sensor_msgs::msg::PointCloud2 &msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2 &msg);
    void RoboSenseHandler(const sensor_msgs::msg::PointCloud2 &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    double time_scale_ = 1e-3;  // Legacy Velodyne scale to milliseconds.
    bool given_offset_time_ = false;

    float height_max_ = 1.0F;
    float height_min_ = -1.0F;
};

}  // namespace lightning

#endif  // LIGHTNING_CORE_LIO_POINTCLOUD_PREPROCESS_H_
