#include "ros/slam_node.h"

#include <yaml-cpp/yaml.h>

#include "lightning/srv/save_map.hpp"
#include "ros/ros_visualization.h"
#include "ros/sensor_bridge.h"

namespace lightning::ros {
namespace {

bool ReadLidarType(const YAML::Node& yaml, LidarType& type) {
    const int value = yaml["fasterlio"]["lidar_type"].as<int>();
    switch (value) {
        case 1:
            type = LidarType::AVIA;
            return true;
        case 2:
            type = LidarType::VELO32;
            return true;
        case 3:
            type = LidarType::OUST64;
            return true;
        case 4:
            type = LidarType::ROBOSENSE;
            return true;
        default:
            return false;
    }
}

}  // namespace

SlamNode::SlamNode(const std::string& config_path, const rclcpp::NodeOptions& options)
    : rclcpp::Node("lightning_slam", options), config_path_(config_path) {}

SlamNode::~SlamNode() { Stop(); }

bool SlamNode::Init() {
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path_);
        if (!ReadLidarType(yaml, lidar_type_)) {
            RCLCPP_ERROR(get_logger(), "unsupported fasterlio.lidar_type");
            return false;
        }
        velodyne_time_scale_ = yaml["fasterlio"]["time_scale"].as<double>();
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(get_logger(), "failed to read config: %s", exception.what());
        return false;
    }

    SlamSystem::Options options;
    options.online_mode_ = true;
    system_ = std::make_shared<SlamSystem>(options);
    if (!system_->Init(config_path_)) {
        system_.reset();
        return false;
    }

    const bool with_ui = yaml["system"]["with_ui"] ? yaml["system"]["with_ui"].as<bool>() : false;
    if (with_ui) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        if (!ui_->Init()) {
            RCLCPP_ERROR(get_logger(), "failed to initialize Pangolin UI");
            ui_.reset();
            system_.reset();
            return false;
        }
        system_->SetNavStateCallback([this](const NavState& state) {
            if (ui_) {
                ui_->UpdateNavState(state);
            }
        });
        system_->SetScanCallback([this](const CloudPtr& cloud, const SE3& pose) {
            if (ui_) {
                ui_->UpdateScan(cloud, pose);
            }
        });
        system_->SetKeyframeCallback([this](const Keyframe::Ptr& keyframe) {
            if (ui_) {
                ui_->UpdateKF(keyframe);
            }
        });
    }

    const bool with_rviz = yaml["system"]["with_rviz"] && yaml["system"]["with_rviz"].as<bool>();
    if (with_rviz) {
        const double publish_hz = yaml["system"]["rviz_local_map_publish_hz"]
                                      ? yaml["system"]["rviz_local_map_publish_hz"].as<double>()
                                      : 2.0;
        const std::size_t max_scans = yaml["system"]["rviz_local_map_max_scans"]
                                          ? yaml["system"]["rviz_local_map_max_scans"].as<std::size_t>()
                                          : 200;
        visualization_ = std::make_shared<RosVisualization>(shared_from_this(), publish_hz, max_scans);
        system_->SetLIODataCallback([this](const LIOData& data) {
            if (visualization_) {
                visualization_->PublishLIOData(data);
            }
        });
        system_->SetGridMapCallback([this](GridMapDataPtr map) {
            if (visualization_) {
                visualization_->PublishGridMap(std::move(map));
            }
        });
    }

    const auto imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    const auto cloud_topic = yaml["common"]["lidar_topic"].as<std::string>();
    const auto livox_topic = yaml["common"]["livox_lidar_topic"].as<std::string>();
    const rclcpp::QoS qos(10);

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { HandleImu(std::move(message)); });
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { HandlePointCloud(std::move(message)); });
    livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic, qos,
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) { HandleLivox(std::move(message)); });

    save_map_service_ = create_service<srv::SaveMap>(
        "lightning/save_map",
        [this](const srv::SaveMap::Request::SharedPtr request,
               srv::SaveMap::Response::SharedPtr response) {
            if (!system_) {
                response->response = -1;
                return;
            }
            system_->SaveMap("./data/" + request->map_id + "/");
            response->response = 0;
        });

    system_->StartSLAM();
    RCLCPP_INFO(get_logger(), "SLAM ROS node has been created");
    return true;
}

void SlamNode::HandleImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
    if (!system_ || !message) {
        return;
    }
    IMUPtr imu;
    if (ToImuData(*message, imu)) {
        system_->ProcessIMU(imu);
    }
}

void SlamNode::HandlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    if (!system_ || !message) {
        return;
    }
    TimedPointCloudData scan;
    if (ToTimedPointCloudData(*message, lidar_type_, velodyne_time_scale_, scan)) {
        system_->ProcessLidar(scan);
    }
}

void SlamNode::HandleLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
    if (!system_ || !message) {
        return;
    }
    TimedPointCloudData scan;
    if (ToTimedPointCloudData(*message, scan)) {
        system_->ProcessLidar(scan);
    }
}

void SlamNode::Stop() {
    if (!system_) {
        return;
    }
    imu_subscription_.reset();
    cloud_subscription_.reset();
    livox_subscription_.reset();
    save_map_service_.reset();
    system_->Stop();
    const auto stats = system_->GetInputStats();
    RCLCPP_INFO(get_logger(),
                "sensor input stats: imu accepted=%zu processed=%zu, scans accepted=%zu processed=%zu, "
                "invalid=%zu time_rejected=%zu queue_full=%zu incomplete=%zu",
                stats.accepted_imu, stats.processed_imu, stats.accepted_scans, stats.processed_scans,
                stats.rejected_invalid, stats.rejected_time, stats.rejected_queue_full,
                stats.incomplete_scans);
    visualization_.reset();
    ui_.reset();
    system_.reset();
}

}  // namespace lightning::ros
