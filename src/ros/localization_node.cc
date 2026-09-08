#include "ros/localization_node.h"

#include <yaml-cpp/yaml.h>

#include "ros/ros_conversions.h"
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

LocalizationNode::LocalizationNode(const std::string& config_path, const rclcpp::NodeOptions& options)
    : rclcpp::Node("lightning_localization", options), config_path_(config_path) {}

LocalizationNode::~LocalizationNode() { Stop(); }

bool LocalizationNode::Init() {
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

    LocSystem::Options options;
    options.pub_tf_ = yaml["system"]["pub_tf"] ? yaml["system"]["pub_tf"].as<bool>() : true;
    system_ = std::make_shared<LocSystem>(options);
    const bool with_ui = yaml["system"]["with_ui"] ? yaml["system"]["with_ui"].as<bool>() : false;
    if (with_ui) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->SetCurrentScanSize(1);
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
        system_->SetRecentPoseCallback([this](const SE3& pose) {
            if (ui_) {
                ui_->UpdateRecentPose(pose);
            }
        });
        system_->SetScanCallback([this](const CloudPtr& cloud, const SE3& pose) {
            if (ui_) {
                ui_->UpdateScan(cloud, pose);
            }
        });
        system_->SetMapUpdateCallback(
            [this](const std::map<int, CloudPtr>& static_cloud,
                   const std::map<int, CloudPtr>& dynamic_cloud) {
                if (ui_) {
                    ui_->UpdatePointCloudGlobal(static_cloud);
                    ui_->UpdatePointCloudDynamic(dynamic_cloud);
                }
            });
    }

    if (!system_->Init(config_path_)) {
        ui_.reset();
        system_.reset();
        return false;
    }

    if (options.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        system_->SetResultCallback([this](const loc::LocalizationResult& result) {
            if (tf_broadcaster_ && result.valid_) {
                tf_broadcaster_->sendTransform(ToTransform(result));
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

    // Preserve the historical online behavior: localization starts from the
    // identity pose unless an application supplies a control API later.
    system_->SetInitPose(SE3());
    RCLCPP_INFO(get_logger(), "localization ROS node has been created");
    return true;
}

void LocalizationNode::HandleImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
    if (!system_ || !message) {
        return;
    }
    IMUPtr imu;
    if (ToImuData(*message, imu)) {
        system_->ProcessIMU(imu);
    }
}

void LocalizationNode::HandlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    if (!system_ || !message) {
        return;
    }
    TimedPointCloudData scan;
    if (ToTimedPointCloudData(*message, lidar_type_, velodyne_time_scale_, scan)) {
        system_->ProcessLidar(scan);
    }
}

void LocalizationNode::HandleLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
    if (!system_ || !message) {
        return;
    }
    TimedPointCloudData scan;
    if (ToTimedPointCloudData(*message, scan)) {
        system_->ProcessLidar(scan);
    }
}

void LocalizationNode::Stop() {
    if (!system_) {
        return;
    }
    imu_subscription_.reset();
    cloud_subscription_.reset();
    livox_subscription_.reset();
    system_->Stop();
    const auto stats = system_->GetInputStats();
    RCLCPP_INFO(get_logger(),
                "sensor input stats: imu accepted=%zu processed=%zu, scans accepted=%zu processed=%zu, "
                "invalid=%zu time_rejected=%zu queue_full=%zu incomplete=%zu",
                stats.accepted_imu, stats.processed_imu, stats.accepted_scans, stats.processed_scans,
                stats.rejected_invalid, stats.rejected_time, stats.rejected_queue_full,
                stats.incomplete_scans);
    tf_broadcaster_.reset();
    ui_.reset();
    system_.reset();
}

}  // namespace lightning::ros
