#include "core/visualization/ros_visualization.h"

#include <pcl_conversions/pcl_conversions.h>

namespace lightning {

RosVisualization::RosVisualization(const rclcpp::Node::SharedPtr& node, double local_map_publish_hz,
                                   std::size_t local_map_max_scans)
    : local_map_publish_period_sec_(local_map_publish_hz > 0.0 ? 1.0 / local_map_publish_hz : 0.0),
      local_map_max_scans_(local_map_max_scans) {
    const rclcpp::QoS path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    lio_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/lio_path", path_qos);
    const rclcpp::QoS sensor_cloud_qos = rclcpp::SensorDataQoS();
    registered_cloud_pub_ =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", sensor_cloud_qos);

    // Keep the latest map so RViz can receive it even when it starts after
    // the first map publication. Depth 1 avoids retaining multiple large maps.
    const rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    grid_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    ivox_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ivox_map", map_qos);
    local_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/local_map", map_qos);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

void RosVisualization::PublishGridMap(nav_msgs::msg::OccupancyGrid map) {
    map.header.frame_id = "map";
    map.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    grid_map_pub_->publish(std::move(map));
}

void RosVisualization::PublishLIOData(const LIOData& data) {
    const rclcpp::Time stamp(static_cast<int64_t>(data.timestamp * 1e9), RCL_ROS_TIME);
    const auto quat = data.pose.unit_quaternion();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map"; // need to be changed!
    pose_msg.pose.position.x = data.pose.translation().x();
    pose_msg.pose.position.y = data.pose.translation().y();
    pose_msg.pose.position.z = data.pose.translation().z();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    // Match Pangolin's green trajectory: append one pose per processed LIO
    // scan, rather than one pose per IMU sample.
    lio_path_.header = pose_msg.header;
    lio_path_.poses.emplace_back(pose_msg);
    constexpr std::size_t kMaxPathPoses = 1000000;
    if (lio_path_.poses.size() > kMaxPathPoses) {
        lio_path_.poses.erase(lio_path_.poses.begin(), lio_path_.poses.begin() + lio_path_.poses.size() / 2);
    }
    lio_path_pub_->publish(lio_path_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose_msg.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pose_msg.pose.position.x;
    transform.transform.translation.y = pose_msg.pose.position.y;
    transform.transform.translation.z = pose_msg.pose.position.z;
    transform.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    if (data.registered_cloud) {
        sensor_msgs::msg::PointCloud2 registered_msg;
        pcl::toROSMsg(*data.registered_cloud, registered_msg);
        registered_msg.header = pose_msg.header;
        registered_cloud_pub_->publish(registered_msg);
    }

    if (data.registered_cloud && local_map_max_scans_ > 0) {
        scan_history_.push_back(data.registered_cloud);
        while (scan_history_.size() > local_map_max_scans_) {
            scan_history_.pop_front();
        }
    }

    const bool map_publish_due =
        local_map_publish_period_sec_ <= 0.0 || last_map_publish_time_.nanoseconds() == 0 ||
        (stamp - last_map_publish_time_).seconds() >= local_map_publish_period_sec_;
    if (map_publish_due) {
        if (data.ivox_map) {
            sensor_msgs::msg::PointCloud2 ivox_map_msg;
            pcl::toROSMsg(*data.ivox_map, ivox_map_msg);
            ivox_map_msg.header = pose_msg.header;
            ivox_map_pub_->publish(ivox_map_msg);
        }

        if (!scan_history_.empty()) {
            PointCloudType local_map;
            std::size_t point_count = 0;
            for (const auto& scan : scan_history_) {
                point_count += scan->size();
            }
            local_map.reserve(point_count);
            for (const auto& scan : scan_history_) {
                local_map += *scan;
            }
            local_map.width = static_cast<uint32_t>(local_map.points.size());
            local_map.height = 1;
            local_map.is_dense = false;

            sensor_msgs::msg::PointCloud2 local_map_msg;
            pcl::toROSMsg(local_map, local_map_msg);
            local_map_msg.header = pose_msg.header;
            local_map_pub_->publish(local_map_msg);
        }
        last_map_publish_time_ = stamp;
    }
}

}  // namespace lightning
