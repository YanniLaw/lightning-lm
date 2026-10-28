#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common/imu.h"
#include "common/odom.h"

namespace lightning::ros {

/// Convert a ROS IMU message into an algorithm-owned value.
bool ToImuData(const sensor_msgs::msg::Imu& message, IMUPtr& output);

// TF requires a valid pose; a valid twist can still be fused without one.
bool IsValidPose(const geometry_msgs::msg::Pose& pose);

// Express wheel-center velocity in IMU axes and correct time before sharing.
// imu_from_wheel maps the wheel frame into the actual IMU frame (T_IB).
bool ToOdom(const nav_msgs::msg::Odometry& message,
            const SE3& imu_from_wheel, double time_offset, OdomPtr& output);

}  // namespace lightning::ros
