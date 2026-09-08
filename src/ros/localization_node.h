#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "core/system/loc_system.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "ui/pangolin_window.h"

namespace lightning::ros {

/// ROS 2 transport and lifecycle wrapper around the localization core.
class LocalizationNode final : public rclcpp::Node {
   public:
    explicit LocalizationNode(const std::string& config_path,
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LocalizationNode() override;

    bool Init();
    void Stop();

   private:
    void HandleImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
    void HandlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
    void HandleLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message);

    std::string config_path_;
    LidarType lidar_type_ = LidarType::AVIA;
    double velodyne_time_scale_ = 1e-3;
    std::shared_ptr<LocSystem> system_;
    std::shared_ptr<ui::PangolinWindow> ui_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_subscription_;
};

}  // namespace lightning::ros
