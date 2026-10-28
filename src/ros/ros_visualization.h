#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/eigen_types.h"
#include "common/grid_map_data.h"
#include "common/pose_graph_data.h"
#include "core/lio/lio_data.h"

namespace lightning::ros {

/// ROS-only visualization consumer for core LIO and pose graph outputs.
class RosVisualization {
   public:
    explicit RosVisualization(const rclcpp::Node::SharedPtr& node, double local_map_publish_hz = 2.0,
                              std::size_t local_map_max_scans = 200);

    void PublishLIOData(const LIOData& data, const SE3& map_from_local);
    // Queues the latest full pose-graph snapshot for ROS publication.
    // Ordinary updates are rate-limited; loop-optimized snapshots are flushed
    // at the next visualization timer tick.
    void PublishPoseGraph(PoseGraphDataPtr data);
    void PublishGridMap(GridMapDataPtr map);

   private:
    void PublishPendingPoseGraph();
    void PublishPoseGraphSnapshot(const PoseGraphData& data);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lio_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr backend_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_graph_node_ids_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ivox_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
    nav_msgs::msg::Path lio_path_;  // History in the frontend's local world.
    rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};
    const double local_map_publish_period_sec_;
    const std::size_t local_map_max_scans_;
    std::deque<std::shared_ptr<const PointCloudType>> scan_history_;

    std::mutex pose_graph_mutex_;
    PoseGraphDataPtr pending_pose_graph_data_;
    bool force_pose_graph_publish_ = false;
    bool has_pose_graph_ = false;
    double last_pose_graph_timestamp_ = 0.0;
    std::uint64_t last_pose_graph_version_ = 0;
    std::size_t last_node_id_marker_count_ = 0;
    rclcpp::TimerBase::SharedPtr pose_graph_publish_timer_;
};

}  // namespace lightning::ros
