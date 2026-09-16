#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>

#include "common/nav_state.h"
#include "common/point_def.h"

namespace lightning::ros {

/// ROS 2 visualization consumer for localization outputs.
class RosLocalizationVisualization final {
   public:
    RosLocalizationVisualization(const rclcpp::Node::SharedPtr& node, double local_map_publish_hz,
                                 std::size_t local_map_max_scans, double path_publish_hz);

    void PublishNavState(const NavState& state);
    void PublishScan(const CloudPtr& cloud, const SE3& pose);
    void PublishMap(const std::map<int, CloudPtr>& static_cloud,
                    const std::map<int, CloudPtr>& dynamic_cloud);

   private:
    static constexpr double kMinPathTranslation = 0.01;
    static constexpr double kMinPathRotation = 0.1 * 3.14159265358979323846 / 180.0;

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PointCloudMessage = sensor_msgs::msg::PointCloud2;

    PoseStamped ToPoseStamped(const SE3& pose, const builtin_interfaces::msg::Time& stamp) const;
    PointCloudMessage ToPointCloudMessage(const PointCloudType& cloud,
                                          const builtin_interfaces::msg::Time& stamp) const;
    PointCloudType::Ptr FlattenClouds(const std::map<int, CloudPtr>& clouds) const;
    PointCloudType::Ptr TransformCloudToMap(const CloudPtr& cloud, const SE3& pose) const;

    builtin_interfaces::msg::Time GetFallbackStamp() const;
    builtin_interfaces::msg::Time GetCloudStamp(const PointCloudType& cloud) const;
    void UpdateLatestStampLocked(const builtin_interfaces::msg::Time& stamp);
    bool ShouldAppendPose(const nav_msgs::msg::Path& path, const SE3& pose) const;
    bool IsPathPublishDueLocked(const rclcpp::Time& stamp, const rclcpp::Time& last_stamp,
                                bool has_last_stamp) const;
    visualization_msgs::msg::MarkerArray BuildPoseMarkersLocked(
        const builtin_interfaces::msg::Time& stamp) const;

    rclcpp::Publisher<PointCloudMessage>::SharedPtr static_map_pub_;
    rclcpp::Publisher<PointCloudMessage>::SharedPtr dynamic_map_pub_;
    rclcpp::Publisher<PointCloudMessage>::SharedPtr current_scan_pub_;
    rclcpp::Publisher<PointCloudMessage>::SharedPtr local_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fused_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lidar_loc_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_markers_pub_;

    rclcpp::Clock::SharedPtr clock_;
    const double local_map_publish_period_sec_;
    const std::size_t local_map_max_scans_;
    const double path_publish_period_sec_;

    // Callbacks can originate from multiple localization worker threads.  A
    // single publication lock preserves snapshot order across all topics.
    mutable std::mutex publish_mutex_;
    mutable std::mutex mutex_;
    nav_msgs::msg::Path fused_path_;
    nav_msgs::msg::Path lidar_loc_path_;
    std::deque<PointCloudType::Ptr> scan_history_;
    rclcpp::Time latest_data_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_local_map_publish_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_fused_path_publish_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lidar_loc_path_publish_stamp_{0, 0, RCL_ROS_TIME};
    bool has_latest_data_stamp_ = false;
    bool has_local_map_publish_stamp_ = false;
    bool has_fused_path_publish_stamp_ = false;
    bool has_lidar_loc_path_publish_stamp_ = false;
    bool has_fused_pose_ = false;
    bool has_lidar_loc_pose_ = false;
    bool fused_marker_published_ = false;
    bool lidar_loc_marker_published_ = false;
    SE3 latest_fused_pose_;
    SE3 latest_lidar_loc_pose_;
};

}  // namespace lightning::ros
