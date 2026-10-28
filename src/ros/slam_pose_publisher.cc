// Copyright 2026

#include "ros/slam_pose_publisher.h"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "ros/ros_conversions.h"
#include "wrapper/ros_utils.h"

namespace lightning::ros {
namespace {

geometry_msgs::msg::TransformStamped MakeTransform(
    double timestamp, const SE3& pose, const std::string& parent,
    const std::string& child) {
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = ToRosTime(timestamp);
  message.header.frame_id = parent;
  message.child_frame_id = child;
  const auto& translation = pose.translation();
  const auto quaternion = pose.unit_quaternion();
  message.transform.translation.x = translation.x();
  message.transform.translation.y = translation.y();
  message.transform.translation.z = translation.z();
  message.transform.rotation.x = quaternion.x();
  message.transform.rotation.y = quaternion.y();
  message.transform.rotation.z = quaternion.z();
  message.transform.rotation.w = quaternion.w();
  return message;
}

}  // namespace

SlamPosePublisher::SlamPosePublisher(const rclcpp::Node::SharedPtr& node,
                                   Options options,
                                   std::optional<SE3> imu_from_wheel)
    : options_(std::move(options)),
      imu_from_wheel_(std::move(imu_from_wheel)) {
  if (options_.publish_tf) {
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
  odometry_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odometry", rclcpp::QoS(10));
  if (broadcaster_ && options_.published_frame != "imu_frame") {
    static_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    static_broadcaster_->sendTransform(MakeTransform(
        node->now().seconds(), imu_from_wheel_->inverse(),
        "center_of_rotation_link", "imu_frame"));
  }
  if (broadcaster_ && options_.published_frame == "odom") {
    odom_tf_subscription_ =
        node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::QoS(100),
            [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
              HandleOdometryTf(std::move(message));
            });
  }
}

void SlamPosePublisher::HandleOdometryTf(
    tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
  for (const auto& transform : message->transforms) {
    if (transform.header.frame_id != "odom" ||
        transform.child_frame_id != "center_of_rotation_link") {
      continue;
    }
    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;
    AddOdometryPose(
        ToSec(transform.header.stamp),
        SE3(Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z),
            Vec3d(translation.x, translation.y, translation.z)));
  }
}

void SlamPosePublisher::AddOdometryPose(double timestamp,
                                      const SE3& odom_from_wheel) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!odom_poses_.empty() && timestamp <= odom_poses_.back().timestamp) return;

  odom_poses_.push_back({timestamp, odom_from_wheel});
  constexpr std::size_t kMaxOdomPoses = 200;
  while (odom_poses_.size() > kMaxOdomPoses) odom_poses_.pop_front();
  PublishPendingPose();
}

void SlamPosePublisher::PublishPose(const LIOState& state,
                                  const SE3& map_from_local) {
  std::lock_guard<std::mutex> lock(mutex_);
  const SE3 local_from_body = options_.published_frame == "imu_frame"
                                  ? state.pose
                                  : state.pose * *imu_from_wheel_;
  PublishOdometry(state, options_.provide_odom_frame
                             ? local_from_body
                             : map_from_local * local_from_body);
  if (!broadcaster_) return;
  if (options_.published_frame == "odom") {
    pending_pose_ = TimedPose{state.timestamp, local_from_body};
    pending_map_from_local_ = map_from_local;
    PublishPendingPose();
    return;
  }
  PublishTransforms(state.timestamp, local_from_body, map_from_local);
}

void SlamPosePublisher::PublishPendingPose() {
  if (!pending_pose_ || odom_poses_.empty()) return;
  const double timestamp = pending_pose_->timestamp;
  if (timestamp < odom_poses_.front().timestamp) {
    pending_pose_.reset();
    return;
  }
  if (timestamp > odom_poses_.back().timestamp) return;

  // Keep the left interpolation endpoint and discard older samples.
  while (odom_poses_.size() > 1 && odom_poses_[1].timestamp <= timestamp) {
    odom_poses_.pop_front();
  }
  const auto& left = odom_poses_.front();
  SE3 odom_from_wheel = left.pose;
  if (timestamp > left.timestamp) {
    const auto& right = odom_poses_[1];
    const double alpha =
        (timestamp - left.timestamp) / (right.timestamp - left.timestamp);
    odom_from_wheel = SE3(
        left.pose.unit_quaternion().slerp(alpha, right.pose.unit_quaternion()),
        (1.0 - alpha) * left.pose.translation() +
            alpha * right.pose.translation());
  }
  const SE3 local_from_odom = pending_pose_->pose * odom_from_wheel.inverse();
  PublishTransforms(timestamp, local_from_odom, pending_map_from_local_);
  pending_pose_.reset();
}

void SlamPosePublisher::PublishTransforms(
    double timestamp, const SE3& local_from_published,
    const SE3& map_from_local) {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  if (options_.provide_odom_frame) {
    transforms.push_back(
        MakeTransform(timestamp, map_from_local, "map", options_.odom_frame));
  }
  transforms.push_back(MakeTransform(
      timestamp,
      options_.provide_odom_frame ? local_from_published
                                 : map_from_local * local_from_published,
      options_.provide_odom_frame ? options_.odom_frame : "map",
      options_.published_frame));
  broadcaster_->sendTransform(transforms);
}

void SlamPosePublisher::PublishOdometry(
    const LIOState& state, const SE3& parent_from_body) {
  nav_msgs::msg::Odometry message;
  message.header.stamp = ToRosTime(state.timestamp);
  message.header.frame_id =
      options_.provide_odom_frame ? options_.odom_frame : "map";
  message.child_frame_id = options_.published_frame == "imu_frame"
                               ? "imu_frame"
                               : "center_of_rotation_link";
  const auto& position = parent_from_body.translation();
  const auto orientation = parent_from_body.unit_quaternion();
  message.pose.pose.position.x = position.x();
  message.pose.pose.position.y = position.y();
  message.pose.pose.position.z = position.z();
  message.pose.pose.orientation.x = orientation.x();
  message.pose.pose.orientation.y = orientation.y();
  message.pose.pose.orientation.z = orientation.z();
  message.pose.pose.orientation.w = orientation.w();
  Vec3d angular_velocity = state.angular_velocity.value_or(Vec3d::Zero());
  if (state.velocity) {
    Vec3d velocity = state.pose.so3().inverse() * *state.velocity;
    if (options_.published_frame != "imu_frame") {
      // Move the velocity origin from the IMU to the wheel center, then
      // express it in wheel axes. T_IB.translation() is the lever arm in I.
      velocity = imu_from_wheel_->so3().inverse() *
                 (velocity + angular_velocity.cross(
                                 imu_from_wheel_->translation()));
    }
    message.twist.twist.linear.x = velocity.x();
    message.twist.twist.linear.y = velocity.y();
    message.twist.twist.linear.z = velocity.z();
  }
  if (state.angular_velocity) {
    if (options_.published_frame != "imu_frame") {
      angular_velocity = imu_from_wheel_->so3().inverse() * angular_velocity;
    }
    message.twist.twist.angular.x = angular_velocity.x();
    message.twist.twist.angular.y = angular_velocity.y();
    message.twist.twist.angular.z = angular_velocity.z();
  }
  odometry_publisher_->publish(message);
}

}  // namespace lightning::ros
