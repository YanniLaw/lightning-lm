#include "ros/ros_localization_visualization.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "ros/ros_conversions.h"

namespace lightning::ros {
namespace {

constexpr char kMapFrame[] = "map";
constexpr char kPoseMarkerNamespace[] = "localization_poses";

visualization_msgs::msg::Marker MakePoseMarker(const std::string& frame_id,
                                               const std::string& marker_namespace, int id,
                                               const builtin_interfaces::msg::Time& stamp,
                                               const SE3& pose, float red, float green, float blue) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.translation().x();
    marker.pose.position.y = pose.translation().y();
    marker.pose.position.z = pose.translation().z();
    const auto quaternion = pose.unit_quaternion();
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();
    marker.scale.x = 1.0;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;
    return marker;
}

visualization_msgs::msg::Marker MakeDeleteMarker(const std::string& frame_id,
                                                 const std::string& marker_namespace, int id,
                                                 const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
}

bool IsFinite(double value) { return std::isfinite(value); }

bool IsFinitePose(const SE3& pose) {
    return pose.translation().allFinite() && pose.unit_quaternion().coeffs().allFinite();
}

builtin_interfaces::msg::Time ToTimeMessage(const rclcpp::Time& time) {
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    const std::int64_t total_nanoseconds = time.nanoseconds();
    std::int64_t seconds = total_nanoseconds / kNanosecondsPerSecond;
    std::int64_t nanoseconds = total_nanoseconds % kNanosecondsPerSecond;
    if (nanoseconds < 0) {
        --seconds;
        nanoseconds += kNanosecondsPerSecond;
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(seconds);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds);
    return stamp;
}

}  // namespace

RosLocalizationVisualization::RosLocalizationVisualization(const rclcpp::Node::SharedPtr& node,
                                                           double local_map_publish_hz,
                                                           std::size_t local_map_max_scans,
                                                           double path_publish_hz)
    : clock_(node ? node->get_clock() : nullptr),
      local_map_publish_period_sec_(local_map_publish_hz > 0.0 ? 1.0 / local_map_publish_hz : 0.0),
      local_map_max_scans_(local_map_max_scans),
      path_publish_period_sec_(path_publish_hz > 0.0 ? 1.0 / path_publish_hz : 0.0) {
    if (!node) {
        throw std::invalid_argument("RosLocalizationVisualization requires a valid ROS node");
    }

    const auto snapshot_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    static_map_pub_ = node->create_publisher<PointCloudMessage>("/localization/map_static", snapshot_qos);
    dynamic_map_pub_ = node->create_publisher<PointCloudMessage>("/localization/map_dynamic", snapshot_qos);
    local_map_pub_ = node->create_publisher<PointCloudMessage>("/localization/local_map", snapshot_qos);
    fused_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/localization/fused_path", snapshot_qos);
    lidar_loc_path_pub_ =
        node->create_publisher<nav_msgs::msg::Path>("/localization/lidar_loc_path", snapshot_qos);
    pose_markers_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>("/localization/poses", snapshot_qos);

    current_scan_pub_ =
        node->create_publisher<PointCloudMessage>("/localization/current_scan", rclcpp::SensorDataQoS());
}

RosLocalizationVisualization::PoseStamped RosLocalizationVisualization::ToPoseStamped(
    const SE3& pose, const builtin_interfaces::msg::Time& stamp) const {
    PoseStamped message;
    message.header.frame_id = kMapFrame;
    message.header.stamp = stamp;
    message.pose.position.x = pose.translation().x();
    message.pose.position.y = pose.translation().y();
    message.pose.position.z = pose.translation().z();
    const auto quaternion = pose.unit_quaternion();
    message.pose.orientation.x = quaternion.x();
    message.pose.orientation.y = quaternion.y();
    message.pose.orientation.z = quaternion.z();
    message.pose.orientation.w = quaternion.w();
    return message;
}

RosLocalizationVisualization::PointCloudMessage RosLocalizationVisualization::ToPointCloudMessage(
    const PointCloudType& cloud, const builtin_interfaces::msg::Time& stamp) const {
    PointCloudMessage message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = kMapFrame;
    message.header.stamp = stamp;
    return message;
}

PointCloudType::Ptr RosLocalizationVisualization::FlattenClouds(
    const std::map<int, CloudPtr>& clouds) const {
    auto flattened = std::make_shared<PointCloudType>();
    std::size_t point_count = 0;
    for (const auto& entry : clouds) {
        if (entry.second) {
            point_count += entry.second->size();
        }
    }
    flattened->reserve(point_count);
    for (const auto& entry : clouds) {
        if (entry.second) {
            *flattened += *entry.second;
        }
    }
    flattened->width = static_cast<std::uint32_t>(flattened->size());
    flattened->height = 1;
    flattened->is_dense = false;
    return flattened;
}

PointCloudType::Ptr RosLocalizationVisualization::TransformCloudToMap(const CloudPtr& cloud,
                                                                       const SE3& pose) const {
    auto transformed = std::make_shared<PointCloudType>();
    transformed->header = cloud->header;
    pcl::transformPointCloud(*cloud, *transformed, pose.matrix());
    transformed->width = static_cast<std::uint32_t>(transformed->size());
    transformed->height = 1;
    transformed->is_dense = false;
    return transformed;
}

builtin_interfaces::msg::Time RosLocalizationVisualization::GetFallbackStamp() const {
    if (clock_) {
        return ToTimeMessage(clock_->now());
    }
    return builtin_interfaces::msg::Time();
}

builtin_interfaces::msg::Time RosLocalizationVisualization::GetCloudStamp(
    const PointCloudType& cloud) const {
    if (cloud.header.stamp != 0) {
        return ToTimeMessage(
            rclcpp::Time(static_cast<std::int64_t>(cloud.header.stamp), RCL_ROS_TIME));
    }
    return GetFallbackStamp();
}

void RosLocalizationVisualization::UpdateLatestStampLocked(
    const builtin_interfaces::msg::Time& stamp) {
    const rclcpp::Time candidate(stamp, RCL_ROS_TIME);
    if (candidate.nanoseconds() <= 0 ||
        (has_latest_data_stamp_ && candidate <= latest_data_stamp_)) {
        return;
    }
    latest_data_stamp_ = candidate;
    has_latest_data_stamp_ = true;
}

bool RosLocalizationVisualization::ShouldAppendPose(const nav_msgs::msg::Path& path,
                                                    const SE3& pose) const {
    if (path.poses.empty()) {
        return true;
    }

    const auto& last_pose = path.poses.back().pose;
    const Eigen::Vector3d last_translation(last_pose.position.x, last_pose.position.y,
                                           last_pose.position.z);
    const Eigen::Quaterniond last_rotation(last_pose.orientation.w, last_pose.orientation.x,
                                           last_pose.orientation.y, last_pose.orientation.z);
    const double translation_delta = (pose.translation() - last_translation).norm();
    const double rotation_delta = pose.unit_quaternion().angularDistance(last_rotation);
    return translation_delta >= kMinPathTranslation || rotation_delta >= kMinPathRotation;
}

bool RosLocalizationVisualization::IsPathPublishDueLocked(const rclcpp::Time& stamp,
                                                           const rclcpp::Time& last_stamp,
                                                           bool has_last_stamp) const {
    return path_publish_period_sec_ <= 0.0 || !has_last_stamp ||
           (stamp > last_stamp &&
            (stamp - last_stamp).seconds() >= path_publish_period_sec_);
}

visualization_msgs::msg::MarkerArray RosLocalizationVisualization::BuildPoseMarkersLocked(
    const builtin_interfaces::msg::Time& stamp) const {
    visualization_msgs::msg::MarkerArray markers;
    if (has_fused_pose_) {
        markers.markers.emplace_back(MakePoseMarker(kMapFrame, kPoseMarkerNamespace, 0, stamp,
                                                    latest_fused_pose_, 1.0F, 0.0F, 0.0F));
    } else if (fused_marker_published_) {
        markers.markers.emplace_back(
            MakeDeleteMarker(kMapFrame, kPoseMarkerNamespace, 0, stamp));
    }

    if (has_lidar_loc_pose_) {
        markers.markers.emplace_back(MakePoseMarker(kMapFrame, kPoseMarkerNamespace, 1, stamp,
                                                    latest_lidar_loc_pose_, 0.0F, 1.0F, 0.0F));
    } else if (lidar_loc_marker_published_) {
        markers.markers.emplace_back(
            MakeDeleteMarker(kMapFrame, kPoseMarkerNamespace, 1, stamp));
    }
    return markers;
}

void RosLocalizationVisualization::PublishNavState(const NavState& state) {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    const auto pose = state.GetPose();
    if (!IsFinite(state.timestamp_) || !IsFinitePose(pose)) {
        return;
    }

    const auto stamp = state.timestamp_ > 0.0 ? ToRosTime(state.timestamp_) : GetFallbackStamp();
    nav_msgs::msg::Path fused_path;
    visualization_msgs::msg::MarkerArray markers;
    bool publish_path = false;
    rclcpp::Time path_stamp(stamp, RCL_ROS_TIME);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        UpdateLatestStampLocked(stamp);
        latest_fused_pose_ = pose;
        has_fused_pose_ = true;

        if (ShouldAppendPose(fused_path_, pose)) {
            fused_path_.header.frame_id = kMapFrame;
            fused_path_.poses.emplace_back(ToPoseStamped(pose, stamp));
        }
        fused_path_.header.frame_id = kMapFrame;
        fused_path_.header.stamp = stamp;

        publish_path = IsPathPublishDueLocked(path_stamp, last_fused_path_publish_stamp_,
                                              has_fused_path_publish_stamp_);
        if (publish_path) {
            fused_path = fused_path_;
            last_fused_path_publish_stamp_ = path_stamp;
            has_fused_path_publish_stamp_ = true;
        }

        markers = BuildPoseMarkersLocked(stamp);
        fused_marker_published_ = has_fused_pose_;
    }

    if (publish_path) {
        fused_path_pub_->publish(std::move(fused_path));
    }
    if (!markers.markers.empty()) {
        pose_markers_pub_->publish(std::move(markers));
    }
}

void RosLocalizationVisualization::PublishScan(const CloudPtr& cloud, const SE3& pose) {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    if (!cloud || cloud->empty() || !IsFinitePose(pose)) {
        return;
    }

    auto transformed = TransformCloudToMap(cloud, pose);
    const auto stamp = GetCloudStamp(*transformed);
    const rclcpp::Time scan_stamp(stamp, RCL_ROS_TIME);
    PointCloudMessage current_scan_message = ToPointCloudMessage(*transformed, stamp);
    PointCloudMessage local_map_message;
    nav_msgs::msg::Path lidar_loc_path;
    visualization_msgs::msg::MarkerArray markers;
    std::deque<PointCloudType::Ptr> local_map_scans;
    bool publish_local_map = false;
    bool publish_path = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        UpdateLatestStampLocked(stamp);
        latest_lidar_loc_pose_ = pose;
        has_lidar_loc_pose_ = true;

        if (local_map_max_scans_ > 0) {
            scan_history_.emplace_back(std::make_shared<PointCloudType>(*transformed));
            while (scan_history_.size() > local_map_max_scans_) {
                scan_history_.pop_front();
            }
        }

        if (ShouldAppendPose(lidar_loc_path_, pose)) {
            lidar_loc_path_.header.frame_id = kMapFrame;
            lidar_loc_path_.poses.emplace_back(ToPoseStamped(pose, stamp));
        }
        lidar_loc_path_.header.frame_id = kMapFrame;
        lidar_loc_path_.header.stamp = stamp;

        publish_local_map = local_map_publish_period_sec_ <= 0.0 || !has_local_map_publish_stamp_ ||
                            (scan_stamp > last_local_map_publish_stamp_ &&
                             (scan_stamp - last_local_map_publish_stamp_).seconds() >=
                                 local_map_publish_period_sec_);
        if (publish_local_map && !scan_history_.empty()) {
            local_map_scans = scan_history_;
            last_local_map_publish_stamp_ = scan_stamp;
            has_local_map_publish_stamp_ = true;
        }

        publish_path = IsPathPublishDueLocked(scan_stamp, last_lidar_loc_path_publish_stamp_,
                                              has_lidar_loc_path_publish_stamp_);
        if (publish_path) {
            lidar_loc_path = lidar_loc_path_;
            last_lidar_loc_path_publish_stamp_ = scan_stamp;
            has_lidar_loc_path_publish_stamp_ = true;
        }

        markers = BuildPoseMarkersLocked(stamp);
        lidar_loc_marker_published_ = has_lidar_loc_pose_;
    }

    if (publish_local_map && !local_map_scans.empty()) {
        PointCloudType local_map;
        std::size_t point_count = 0;
        for (const auto& scan : local_map_scans) {
            point_count += scan->size();
        }
        local_map.reserve(point_count);
        for (const auto& scan : local_map_scans) {
            local_map += *scan;
        }
        local_map.width = static_cast<std::uint32_t>(local_map.size());
        local_map.height = 1;
        local_map.is_dense = false;
        local_map_message = ToPointCloudMessage(local_map, stamp);
    }

    current_scan_pub_->publish(std::move(current_scan_message));
    if (publish_local_map && !local_map_message.data.empty()) {
        local_map_pub_->publish(std::move(local_map_message));
    }
    if (publish_path) {
        lidar_loc_path_pub_->publish(std::move(lidar_loc_path));
    }
    if (!markers.markers.empty()) {
        pose_markers_pub_->publish(std::move(markers));
    }
}

void RosLocalizationVisualization::PublishMap(const std::map<int, CloudPtr>& static_cloud,
                                               const std::map<int, CloudPtr>& dynamic_cloud) {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    const auto static_map = FlattenClouds(static_cloud);
    const auto dynamic_map = FlattenClouds(dynamic_cloud);

    builtin_interfaces::msg::Time stamp;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stamp = has_latest_data_stamp_ ? ToTimeMessage(latest_data_stamp_) : GetFallbackStamp();
    }

    static_map_pub_->publish(ToPointCloudMessage(*static_map, stamp));
    dynamic_map_pub_->publish(ToPointCloudMessage(*dynamic_map, stamp));
}

}  // namespace lightning::ros
