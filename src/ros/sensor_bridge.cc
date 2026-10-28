#include "ros/sensor_bridge.h"

#include <cmath>
#include <limits>
#include <utility>

#include "wrapper/ros_utils.h"

namespace lightning::ros {
namespace {

bool IsFinite(const geometry_msgs::msg::Vector3& vector) {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

bool IsFinite(const geometry_msgs::msg::Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool IsFinite(const geometry_msgs::msg::Quaternion& quaternion) {
    return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) && std::isfinite(quaternion.z) &&
           std::isfinite(quaternion.w);
}

}  // namespace

bool IsValidPose(const geometry_msgs::msg::Pose& pose) {
  const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                      pose.orientation.y, pose.orientation.z);
  return IsFinite(pose.position) && IsFinite(pose.orientation) &&
         std::isfinite(quaternion.norm()) &&
         quaternion.norm() > std::numeric_limits<double>::epsilon();
}

bool ToImuData(const sensor_msgs::msg::Imu& message, IMUPtr& output) {
    const double timestamp = ToSec(message.header.stamp);
    if (!std::isfinite(timestamp) || !std::isfinite(message.angular_velocity.x) ||
        !std::isfinite(message.angular_velocity.y) || !std::isfinite(message.angular_velocity.z) ||
        !std::isfinite(message.linear_acceleration.x) || !std::isfinite(message.linear_acceleration.y) ||
        !std::isfinite(message.linear_acceleration.z)) {
        return false;
    }

    IMUPtr converted = std::make_shared<IMU>();
    converted->timestamp = timestamp;
    converted->angular_velocity =
        Vec3d(message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
    converted->linear_acceleration =
        Vec3d(message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z);
    output = std::move(converted);
    return true;
}

bool ToOdom(const nav_msgs::msg::Odometry& message, const SE3& imu_from_wheel,
            double time_offset, OdomPtr& output) {
  const double timestamp = ToSec(message.header.stamp) + time_offset;
  if (!std::isfinite(timestamp) || timestamp <= 0.0 ||
      !IsFinite(message.twist.twist.linear) ||
      !IsFinite(message.twist.twist.angular)) {
    return false;
  }

  const Eigen::Quaterniond quaternion(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
  auto converted = std::make_shared<Odom>();
  converted->timestamp = timestamp;
  if (IsValidPose(message.pose.pose)) {
    converted->pose =
        SE3(quaternion.normalized(),
            Vec3d(message.pose.pose.position.x, message.pose.pose.position.y,
                  message.pose.pose.position.z)) * imu_from_wheel.inverse();
  }
  converted->linear_velocity = imu_from_wheel.so3() *
      Vec3d(message.twist.twist.linear.x, message.twist.twist.linear.y,
            message.twist.twist.linear.z);
  converted->angular_velocity = imu_from_wheel.so3() *
      Vec3d(message.twist.twist.angular.x, message.twist.twist.angular.y,
            message.twist.twist.angular.z);
  output = std::move(converted);
  return true;
}

}  // namespace lightning::ros
