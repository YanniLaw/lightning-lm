// Copyright 2026

#ifndef LIGHTNING_ROS_SLAM_POSE_PUBLISHER_H_
#define LIGHTNING_ROS_SLAM_POSE_PUBLISHER_H_

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "common/eigen_types.h"
#include "core/lio/lio_result.h"

namespace lightning::ros {

// Publishes Cartographer-style TF and independent LIO body odometry.
// Chassis TF always uses unshifted ROS header seconds.
class SlamPosePublisher {
 public:
  struct Options {
    bool publish_tf = true;
    bool provide_odom_frame = false;
    std::string odom_frame = "odom";
    std::string published_frame = "imu_frame";
  };

  SlamPosePublisher(const rclcpp::Node::SharedPtr& node, Options options,
                    std::optional<SE3> imu_from_wheel);

  // The caller fixes the backend correction for this scan before calling.
  void PublishPose(const LIOState& state, const SE3& map_from_local);

 private:
  struct TimedPose {
    double timestamp;
    SE3 pose;
  };

  void HandleOdometryTf(tf2_msgs::msg::TFMessage::ConstSharedPtr message);
  void AddOdometryPose(double timestamp, const SE3& odom_from_wheel);
  // Called with mutex_ held. Interpolate at the wheel center before moving
  // to the published frame, preserving rotation of the installation lever arm.
  void PublishPendingPose();
  void PublishTransforms(double timestamp, const SE3& local_from_published,
                         const SE3& map_from_local);
  void PublishOdometry(const LIOState& state, const SE3& parent_from_body);

  const Options options_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odom_tf_subscription_;
  const std::optional<SE3> imu_from_wheel_;
  std::mutex mutex_;
  std::deque<TimedPose> odom_poses_;  // T_OB, bounded to two seconds at 100 Hz.
  std::optional<TimedPose> pending_pose_;  // Latest scan's fixed T_LB.
  SE3 pending_map_from_local_;  // Same scan's correction; used while pending.
};

}  // namespace lightning::ros

#endif  // LIGHTNING_ROS_SLAM_POSE_PUBLISHER_H_
