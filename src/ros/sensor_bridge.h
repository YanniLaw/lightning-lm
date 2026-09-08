#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/imu.h"
#include "common/sensor_data.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning::ros {

/// Convert a ROS IMU message into an algorithm-owned value.
bool ToImuData(const sensor_msgs::msg::Imu& message, IMUPtr& output);

/// Convert a Livox custom scan into the transport-independent representation.
bool ToTimedPointCloudData(const livox_ros_driver2::msg::CustomMsg& message,
                           TimedPointCloudData& output);

/// Convert a standard PointCloud2 scan.  The Velodyne scale is in milliseconds
/// per message time unit and is applied exactly once at this boundary.
bool ToTimedPointCloudData(const sensor_msgs::msg::PointCloud2& message, LidarType lidar_type,
                           double velodyne_time_scale, TimedPointCloudData& output);

}  // namespace lightning::ros
