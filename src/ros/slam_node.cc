#include "ros/slam_node.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <mutex>
#include <utility>

#include "io/wheel_odometry_options.h"
#include "lightning/srv/save_map.hpp"
#include "ros/ros_visualization.h"
#include "ros/sensor_bridge.h"

namespace lightning::ros {

SlamNode::SlamNode(const std::string& config_path, const rclcpp::NodeOptions& options)
    : rclcpp::Node("lightning_slam", options), config_path_(config_path) {}

SlamNode::~SlamNode() { Stop(); }

bool SlamNode::Init() {
    YAML::Node yaml;
    bool publish_tf = true;
    try {
        yaml = YAML::LoadFile(config_path_);
        if (!preprocess_.Init(yaml)) return false;
        std::string error;
        if (!ParseWheelOdometryOptions(yaml, wheel_options_, error)) {
          LOG(ERROR) << error;
          return false;
        }
        if (wheel_options_.fusion_enabled) {
          imu_from_wheel_ = preprocess_.GetImuFromLidar() *
                            wheel_options_.lidar_from_wheel;
        }
        if (yaml["system"]["pub_tf"]) {
          publish_tf = yaml["system"]["pub_tf"].as<bool>();
        }
    } catch (const std::exception& exception) {
        LOG(ERROR) << "failed to read config: " << exception.what();
        return false;
    }

    stopping_.store(false);

    SlamSystem::Options options;
    options.online_mode_ = true;
    system_ = std::make_shared<SlamSystem>(options);
    if (!system_->Init(config_path_)) {
        system_.reset();
        return false;
    }

    const bool with_ui = yaml["system"]["with_ui"] ? yaml["system"]["with_ui"].as<bool>() : false;
    if (with_ui) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        if (!ui_->Init()) {
            LOG(ERROR) << "failed to initialize Pangolin UI";
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
        visualization_->SetPublishTf(publish_tf);
        system_->SetLIODataCallback([this](const LIOData& data) {
            if (visualization_) {
                visualization_->PublishLIOData(data);
            }
        });
        system_->SetPoseGraphDataCallback([this](PoseGraphDataPtr data) {
            if (visualization_) {
                visualization_->PublishPoseGraph(std::move(data));
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

    if (wheel_options_.fusion_enabled) {
        const std::string wheel_odometry_topic = yaml["common"]["wheel_odom_topic"].as<std::string>();
        wheel_odometry_subscription_ =
            create_subscription<nav_msgs::msg::Odometry>(
                wheel_odometry_topic, rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                  HandleWheelOdometry(std::move(message));
                });
        LOG(INFO) << "wheel odometry input enabled on " << wheel_odometry_topic;
    }

    save_map_service_ = create_service<srv::SaveMap>(
        "lightning/save_map",
        [this](const srv::SaveMap::Request::SharedPtr request,
               srv::SaveMap::Response::SharedPtr response) {
            if (!system_) {
                LOG(WARNING) << "SLAM system not exitst!";
                response->response = -1;
                return;
            }
            system_->SaveMap("./data/" + request->map_id + "/");
            response->response = 0;
        });

    system_->StartSLAM();
    LOG(INFO) << "SLAM ROS node has been created";
    return true;
}

void SlamNode::HandleImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
  if (stopping_.load() || !system_ || !message) {
    return;
  }
    IMUPtr imu;
    if (ToImuData(*message, imu)) {
        system_->ProcessIMU(imu);
    }
}

void SlamNode::HandlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
  if (stopping_.load() || !system_ || !message) {
    return;
  }
    CloudPtr scan;
    if (preprocess_.Process(*message, scan)) {
      system_->ProcessLidar(scan);
    }
}

void SlamNode::HandleLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
  if (stopping_.load() || !system_ || !message) {
    return;
  }
    CloudPtr scan;
    if (preprocess_.Process(*message, scan)) {
      system_->ProcessLidar(scan);
    }
}

void SlamNode::HandleWheelOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(wheel_input_mutex_);
  if (stopping_.load() || !system_ || !message) {
    return;
  }

  OdomPtr data;
  if (!ToOdom(*message, imu_from_wheel_, wheel_options_.wheel_time_offset, data)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Reject invalid wheel odometry");
    return;
  }
  const LIOInputStatus status = system_->ProcessWheelOdometry(data);
  if (status != LIOInputStatus::kAccepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Wheel odometry rejected: status=%d",
                         static_cast<int>(status));
  }
}

void SlamNode::Stop() {
    stopping_.store(true);
    if (!system_) {
        LOG(WARNING) << "System is not initialized, stopping SlamNode.";
        return;
    }
    imu_subscription_.reset();
    cloud_subscription_.reset();
    livox_subscription_.reset();
    wheel_odometry_subscription_.reset();
    save_map_service_.reset();
    system_->Stop();
    const auto stats = system_->GetInputStats();
    LOG(INFO) << "sensor input stats: imu accepted=" << stats.accepted_imu
              << " processed=" << stats.processed_imu << ", scans accepted=" << stats.accepted_scans
              << " processed=" << stats.processed_scans << ", invalid=" << stats.rejected_invalid
              << " time_rejected=" << stats.rejected_time << " queue_full=" << stats.rejected_queue_full;
    LOG(INFO) << "wheel input stats: accepted=" << stats.accepted_wheel
              << " processed=" << stats.processed_wheel
              << " dropped=" << stats.dropped_wheel
              << " cancelled=" << stats.cancelled_wheel
              << " invalid=" << stats.invalid_wheel
              << " time_rejected=" << stats.time_rejected_wheel;
    visualization_.reset();
    ui_.reset();
    system_.reset();
}

}  // namespace lightning::ros
