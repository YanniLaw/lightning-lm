#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common/imu.h"
#include "common/odom.h"

namespace lightning::ros {

/// Convert a ROS IMU message into an algorithm-owned value.
bool ToImuData(const sensor_msgs::msg::Imu& message, IMUPtr& output);

/// Convert a wheel-encoder odometry message without applying frame transforms
/// or feeding it into an estimator. Message covariance is ignored.
bool ToOdom(const nav_msgs::msg::Odometry& message, Odom& output);

}  // namespace lightning::ros
