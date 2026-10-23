#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "common/sensor_data.h"
#include "common/wheel_odometry_options.h"
#include "core/system/slam.h"
#include "lightning/srv/save_map.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "core/lio/pointcloud_preprocess.h"
#include "ui/pangolin_window.h"

namespace lightning::ros {

class RosVisualization;

/// ROS 2 transport and lifecycle wrapper around the transport-independent SLAM core.
class SlamNode final : public rclcpp::Node {
   public:
    explicit SlamNode(const std::string& config_path, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SlamNode() override;

    bool Init();
    void Stop();

   private:
    void HandleImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
    void HandlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
    void HandleLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message);
    void HandleWheelOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message);

    std::string config_path_;
    PointCloudPreprocess preprocess_;
    WheelOdometryOptions wheel_options_;
    std::mutex wheel_input_mutex_;
    std::atomic_bool stopping_ = false;
    std::shared_ptr<SlamSystem> system_;
    std::shared_ptr<ui::PangolinWindow> ui_;
    std::shared_ptr<RosVisualization> visualization_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_subscription_;
    rclcpp::Service<srv::SaveMap>::SharedPtr save_map_service_;
};

}  // namespace lightning::ros
