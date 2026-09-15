#include "ros/ros_visualization.h"

#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <unordered_map>
#include <utility>

#include "ros/ros_conversions.h"

namespace lightning::ros {
namespace {

geometry_msgs::msg::PoseStamped ToPoseStamped(const SE3& pose, double timestamp) {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = ToRosTime(timestamp);
    message.header.frame_id = "map";

    const auto& translation = pose.translation();
    const auto quaternion = pose.unit_quaternion();
    message.pose.position.x = translation.x();
    message.pose.position.y = translation.y();
    message.pose.position.z = translation.z();
    message.pose.orientation.x = quaternion.x();
    message.pose.orientation.y = quaternion.y();
    message.pose.orientation.z = quaternion.z();
    message.pose.orientation.w = quaternion.w();
    return message;
}

geometry_msgs::msg::Point ToPoint(const SE3& pose) {
    geometry_msgs::msg::Point point;
    const auto& translation = pose.translation();
    point.x = translation.x();
    point.y = translation.y();
    point.z = translation.z();
    return point;
}

visualization_msgs::msg::Marker MakeMarker(const std::string& marker_namespace, int id, int type,
                                           const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
}

visualization_msgs::msg::Marker MakeDeleteMarker(const std::string& marker_namespace, int id,
                                                 const builtin_interfaces::msg::Time& stamp) {
    auto marker = MakeMarker(marker_namespace, id, visualization_msgs::msg::Marker::CUBE, stamp);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
}

}  // namespace

RosVisualization::RosVisualization(const rclcpp::Node::SharedPtr& node, double local_map_publish_hz,
                                   std::size_t local_map_max_scans)
    : local_map_publish_period_sec_(local_map_publish_hz > 0.0 ? 1.0 / local_map_publish_hz : 0.0),
      local_map_max_scans_(local_map_max_scans) {
    const rclcpp::QoS path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    lio_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/lio_path", path_qos);
    backend_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/backend_path", path_qos);
    pose_graph_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>("/pose_graph", path_qos);
    pose_graph_node_ids_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pose_graph_node_ids", path_qos);

    const rclcpp::QoS sensor_cloud_qos = rclcpp::SensorDataQoS();
    registered_cloud_pub_ =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", sensor_cloud_qos);

    const rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    grid_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    ivox_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ivox_map", map_qos);
    local_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/local_map", map_qos);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    pose_graph_publish_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { PublishPendingPoseGraph(); });
}

void RosVisualization::PublishGridMap(GridMapDataPtr map) {
    if (!map) {
        return;
    }
    auto message = ToOccupancyGrid(*map);
    message.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    grid_map_pub_->publish(std::move(message));
}

void RosVisualization::PublishPoseGraph(PoseGraphDataPtr data) {
    if (!data || data->nodes.empty()) {
        return;
    }

    const bool optimized = data->optimized;
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (data->version <= last_pose_graph_version_) {
        return;
    }
    if (!pending_pose_graph_data_ || pending_pose_graph_data_->version < data->version) {
        pending_pose_graph_data_ = std::move(data);
    }
    force_pose_graph_publish_ = force_pose_graph_publish_ || optimized;
}

void RosVisualization::PublishPendingPoseGraph() {
    PoseGraphDataPtr data;
    {
        std::lock_guard<std::mutex> lock(pose_graph_mutex_);
        if (!pending_pose_graph_data_ ||
            pending_pose_graph_data_->version <= last_pose_graph_version_) {
            pending_pose_graph_data_.reset();
            force_pose_graph_publish_ = false;
            return;
        }

        const double latest_timestamp = pending_pose_graph_data_->nodes.back().timestamp;
        constexpr double kPoseGraphPublishPeriodSec = 1.0;
        const bool publish_due =
            !has_pose_graph_ || force_pose_graph_publish_ ||
            latest_timestamp - last_pose_graph_timestamp_ >= kPoseGraphPublishPeriodSec;
        if (!publish_due) {
            return;
        }

        data = std::move(pending_pose_graph_data_);
        force_pose_graph_publish_ = false;
    }

    PublishPoseGraphSnapshot(*data);

    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    last_pose_graph_version_ = data->version;
    last_pose_graph_timestamp_ = data->nodes.back().timestamp;
    has_pose_graph_ = true;
}

void RosVisualization::PublishPoseGraphSnapshot(const PoseGraphData& data) {
    if (data.nodes.empty()) {
        return;
    }

    const double latest_timestamp = data.nodes.back().timestamp;
    const auto stamp = ToRosTime(latest_timestamp);

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = stamp;
    path.poses.reserve(data.nodes.size());
    for (const auto& node : data.nodes) {
        path.poses.emplace_back(ToPoseStamped(node.pose, node.timestamp));
    }
    backend_path_pub_->publish(std::move(path));

    std::unordered_map<unsigned long, geometry_msgs::msg::Point> node_points;
    node_points.reserve(data.nodes.size());

    visualization_msgs::msg::MarkerArray graph_markers;
    auto nodes_marker = MakeMarker("pose_graph_nodes", 0,
                                   visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
    nodes_marker.scale.x = 0.1;
    nodes_marker.scale.y = 0.1;
    nodes_marker.scale.z = 0.1;
    nodes_marker.color.r = 0.1F;
    nodes_marker.color.g = 0.7F;
    nodes_marker.color.b = 1.0F;
    nodes_marker.color.a = 1.0F;
    nodes_marker.points.reserve(data.nodes.size());
    for (const auto& node : data.nodes) {
        const auto point = ToPoint(node.pose);
        node_points.emplace(node.id, point);
        nodes_marker.points.emplace_back(point);
    }
    graph_markers.markers.emplace_back(std::move(nodes_marker));

    auto loop_marker = MakeMarker("pose_graph_loop_constraints", 0,
                                  visualization_msgs::msg::Marker::LINE_LIST, stamp);
    loop_marker.scale.x = 0.06;
    loop_marker.color.r = 1.0F;
    loop_marker.color.g = 1.0F;
    loop_marker.color.b = 0.0F;
    loop_marker.color.a = 1.0F;

    auto outlier_marker = MakeMarker("pose_graph_loop_outliers", 1,
                                     visualization_msgs::msg::Marker::LINE_LIST, stamp);
    outlier_marker.scale.x = 0.06;
    outlier_marker.color.r = 1.0F;
    outlier_marker.color.g = 0.1F;
    outlier_marker.color.b = 0.1F;
    outlier_marker.color.a = 1.0F;

    for (const auto& constraint : data.loop_constraints) {
        const auto from = node_points.find(constraint.from_id);
        const auto to = node_points.find(constraint.to_id);
        if (from == node_points.end() || to == node_points.end()) {
            continue;
        }
        auto& marker = constraint.is_outlier ? outlier_marker : loop_marker;
        marker.points.emplace_back(from->second);
        marker.points.emplace_back(to->second);
    }

    if (loop_marker.points.empty()) {
        graph_markers.markers.emplace_back(
            MakeDeleteMarker("pose_graph_loop_constraints", 0, stamp));
    } else {
        graph_markers.markers.emplace_back(std::move(loop_marker));
    }
    if (outlier_marker.points.empty()) {
        graph_markers.markers.emplace_back(
            MakeDeleteMarker("pose_graph_loop_outliers", 1, stamp));
    } else {
        graph_markers.markers.emplace_back(std::move(outlier_marker));
    }
    pose_graph_pub_->publish(std::move(graph_markers));

    visualization_msgs::msg::MarkerArray node_id_markers;
    node_id_markers.markers.reserve(data.nodes.size() + last_node_id_marker_count_);
    for (std::size_t index = 0; index < data.nodes.size(); ++index) {
        const auto& node = data.nodes[index];
        auto marker = MakeMarker("pose_graph_node_ids", static_cast<int>(index),
                                 visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
        marker.pose.position = node_points.at(node.id);
        marker.pose.position.z += 0.35;
        marker.scale.z = 0.25;
        marker.color.r = 1.0F;
        marker.color.g = 1.0F;
        marker.color.b = 1.0F;
        marker.color.a = 1.0F;
        marker.text = std::to_string(node.id);
        node_id_markers.markers.emplace_back(std::move(marker));
    }
    for (std::size_t index = data.nodes.size(); index < last_node_id_marker_count_; ++index) {
        node_id_markers.markers.emplace_back(
            MakeDeleteMarker("pose_graph_node_ids", static_cast<int>(index), stamp));
    }
    pose_graph_node_ids_pub_->publish(std::move(node_id_markers));
    last_node_id_marker_count_ = data.nodes.size();
}

void RosVisualization::PublishLIOData(const LIOData& data) {
    const rclcpp::Time stamp(static_cast<int64_t>(data.timestamp * 1e9), RCL_ROS_TIME);
    const auto quat = data.pose.unit_quaternion();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = data.pose.translation().x();
    pose_msg.pose.position.y = data.pose.translation().y();
    pose_msg.pose.position.z = data.pose.translation().z();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

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
    if (!map_publish_due) {
        return;
    }

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

}  // namespace lightning::ros
