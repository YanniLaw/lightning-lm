#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <deque>
#include <utility>

#include "core/lio/lio_data.h"

namespace lightning {

class RosVisualization {
   public:
    explicit RosVisualization(const rclcpp::Node::SharedPtr& node, double local_map_publish_hz = 2.0,
                              std::size_t local_map_max_scans = 200);

    void PublishLIOData(const LIOData& data);
    void PublishGridMap(nav_msgs::msg::OccupancyGrid map);

   private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lio_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ivox_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path lio_path_;
    rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};
    const double local_map_publish_period_sec_;
    const std::size_t local_map_max_scans_;
    std::deque<std::shared_ptr<const PointCloudType>> scan_history_;
};

}  // namespace lightning
