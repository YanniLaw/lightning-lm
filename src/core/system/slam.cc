//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/lio_factory.h"
#include "core/lio/lio_result_conversion.h"
#include "core/maps/keyframe_map.h"
#include "core/maps/tiled_map.h"

#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    sys::SensorDispatcher::Options dispatcher_options;
    sensor_dispatcher_ = std::make_unique<sys::SensorDispatcher>(
        dispatcher_options,
        [this](const IMUPtr& imu) { ProcessIMUOnWorker(imu); },
        [this](const TimedPointCloudData& cloud) { ProcessLidarOnWorker(cloud); });
}

bool SlamSystem::Init(const std::string& yaml_path) {
    auto yaml = YAML::LoadFile(yaml_path);
    std::string lio_type = "aa_fasterlio";
    if (yaml["system"] && yaml["system"]["lio_type"]) {
        lio_type = yaml["system"]["lio_type"].as<std::string>();
    }

    lio_ = CreateLIO(lio_type);
    if (!lio_) {
        LOG(ERROR) << "failed to create LIO frontend: " << lio_type;
        return false;
    }

    lio_->SetResultCallback([this](const LIOResult& result) { HandleLIOResult(result); });
    lio_->SetPredictionCallback([this](const LIOState& state) { HandleLIOPrediction(state); });
    lio_->SetDataCallback(lio_data_callback_);

    LIOOptions lio_options;
    lio_options.usage = LIOUsage::kMapping;
    if (!lio_->Init(yaml_path, lio_options)) {
        LOG(ERROR) << "failed to init lio module";
        lio_.reset();
        return false;
    }

    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    options_.with_rviz_ = yaml["system"]["with_rviz"] ? yaml["system"]["with_rviz"].as<bool>() : false;
    options_.rviz_local_map_publish_hz_ = yaml["system"]["rviz_local_map_publish_hz"]
                                             ? yaml["system"]["rviz_local_map_publish_hz"].as<double>()
                                             : 2.0;
    options_.rviz_local_map_max_scans_ = yaml["system"]["rviz_local_map_max_scans"]
                                             ? yaml["system"]["rviz_local_map_max_scans"].as<std::size_t>()
                                             : 200;

    LOG(INFO) << "slam with loop closing: " << std::boolalpha << options_.with_loop_closing_;
    PoseGraph::Options pose_graph_options;
    pose_graph_options.online_mode_ = options_.online_mode_;
    pose_graph_options.enable_loop_closing_ = options_.with_loop_closing_;
    pose_graph_ = std::make_shared<PoseGraph>(pose_graph_options);
    pose_graph_->Init(yaml_path);
    if (pose_graph_data_callback_) {
        pose_graph_->SetDataCallback(pose_graph_data_callback_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_ && pose_graph_) {
            /// 当发生回环时，触发一次重绘
            pose_graph_->SetOptimizedCallback([this]() { g2p5_->RedrawGlobalMap(); });
        }

        g2p5_->SetMapUpdateCallback([this](GridMapDataPtr map) {
            if (grid_map_callback_) {
                grid_map_callback_(std::move(map));
            }
        });
    }

    return true;
}

SlamSystem::~SlamSystem() { Stop(); }

void SlamSystem::StartSLAM(/*std::string map_name*/) {
    map_name_ = "new_map";
    running_ = true;
    if (options_.online_mode_ && sensor_dispatcher_) {
        sensor_dispatcher_->Start();
    }
    LOG(INFO) << "SLAM started.";
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
        LOG(WARNING) << "save path is empty, using default path: " << save_path;
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    if (!pose_graph_) {
        LOG(ERROR) << "pose graph is not initialized, cannot save map";
        return;
    }

    const auto keyframes = pose_graph_->GetMapSnapshot();
    if (keyframes.empty()) {
        LOG(WARNING) << "no keyframes available, map was not saved";
        return;
    }

    auto global_map = BuildKeyframeMap(keyframes, !options_.with_loop_closing_);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    const SE3 start_pose = options_.with_loop_closing_ ? keyframes.front().optimized_pose
                                                       : keyframes.front().lio_pose;
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    LOG(INFO) << "Saved global map to " << save_path + "/global.pcd";
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        auto map = g2p5_->GetNewestMap();
        if (!map) {
            LOG(WARNING) << "no grid map is available";
            return;
        }
        auto grid = map->ToGridData();
        const int width = static_cast<int>(grid->width);
        const int height = static_cast<int>(grid->height);

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = grid->cells[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << grid->width;
            emitter << YAML::Key << "height" << YAML::Value << grid->height;
            emitter << YAML::Key << "resolution" << YAML::Value << grid->resolution;
            std::vector<double> orig{grid->origin.x(), grid->origin.y(), 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (!running_ || !lio_ || !imu) {
        return;
    }
    if (!options_.online_mode_ || !sensor_dispatcher_) {
        ProcessIMUOnWorker(imu);
        return;
    }

    const InputResult result = sensor_dispatcher_->AddImu(imu);
    if (result != InputResult::Accepted && result != InputResult::NotRunning) {
        LOG(WARNING) << "reject IMU input: " << static_cast<int>(result);
    }
}

bool SlamSystem::ProcessLidar(const TimedPointCloudData& cloud) {
    if (!running_ || !lio_) {
        return false;
    }

    if (!options_.online_mode_ || !sensor_dispatcher_) {
        return ProcessLidarOnWorker(cloud);
    }

    auto scan = std::make_shared<TimedPointCloudData>(cloud);
    const InputResult result = sensor_dispatcher_->AddPointCloud(std::move(scan));
    if (result != InputResult::Accepted && result != InputResult::NotRunning) {
        LOG(WARNING) << "reject lidar input: " << static_cast<int>(result);
    }
    return result == InputResult::Accepted;
}

void SlamSystem::ProcessIMUOnWorker(const IMUPtr& imu) {
    if (!lio_ || !imu) {
        return;
    }
    lio_->AddImu(imu);
}

bool SlamSystem::ProcessLidarOnWorker(const TimedPointCloudData& cloud) {
    if (!lio_) {
        return false;
    }
    const LIOInputStatus result = lio_->AddPointCloud(cloud);
    return result == LIOInputStatus::kAccepted;
}

void SlamSystem::HandleLIOPrediction(const LIOState& state) {
    NavState nav_state;
    if (!ConvertToNavState(state, nav_state)) {
        return;
    }
    if (nav_state_callback_) {
        nav_state_callback_(nav_state);
    }
}

void SlamSystem::HandleLIOResult(const LIOResult& result) {
    if (!result.state.pose_is_valid || !result.state.pose.matrix().allFinite()) {
        return;
    }

    if (scan_callback_ && result.display_cloud) {
        auto display_cloud = std::make_shared<PointCloudType>(*result.display_cloud);
        scan_callback_(std::move(display_cloud), result.state.pose);
    }

    if (result.update_type != LIOUpdateType::kScanMatched || !result.keyframe_selected || !pose_graph_) {
        return;
    }

    auto keyframe = pose_graph_->AddKeyframe(result);
    if (!keyframe) {
        LOG(WARNING) << "LIO keyframe result was rejected by pose graph";
        return;
    }

    if (options_.with_gridmap_ && g2p5_) {
        g2p5_->PushKeyframe(keyframe);
    }
    if (keyframe_callback_) {
        keyframe_callback_(keyframe);
    }
}

LIOInputStatus SlamSystem::ProcessWheelOdometry(const WheelOdometryData& data) {
    if (!running_ || !lio_) {
        return LIOInputStatus::kNotRunning;
    }
    const LIOInputStatus lio_status = lio_->AddWheelOdometry(data);
    if (pose_graph_) {
        pose_graph_->AddWheelOdometry(data);
    }
    return lio_status;
}

void SlamSystem::SetLIODataCallback(std::function<void(const LIOData&)> callback) {
    lio_data_callback_ = std::move(callback);
    if (lio_) {
        lio_->SetDataCallback(lio_data_callback_);
    }
}

void SlamSystem::SetNavStateCallback(std::function<void(const NavState&)> callback) {
    nav_state_callback_ = std::move(callback);
}

void SlamSystem::SetScanCallback(std::function<void(const CloudPtr&, const SE3&)> callback) {
    scan_callback_ = std::move(callback);
}

void SlamSystem::SetKeyframeCallback(std::function<void(const Keyframe::Ptr&)> callback) {
    keyframe_callback_ = std::move(callback);
}

void SlamSystem::SetGridMapCallback(std::function<void(GridMapDataPtr)> callback) {
    grid_map_callback_ = std::move(callback);
}

void SlamSystem::SetPoseGraphDataCallback(std::function<void(PoseGraphDataPtr)> callback) {
    pose_graph_data_callback_ = std::move(callback);
    if (pose_graph_) {
        pose_graph_->SetDataCallback(pose_graph_data_callback_);
    }
}

sys::SensorDispatcher::Stats SlamSystem::GetInputStats() const {
    return sensor_dispatcher_ ? sensor_dispatcher_->GetStats() : sys::SensorDispatcher::Stats();
}

void SlamSystem::Stop() {
    if (options_.online_mode_ && sensor_dispatcher_) {
        sensor_dispatcher_->Stop(sys::SensorDispatcher::StopMode::kDrain);
    }
    if (pose_graph_) {
        pose_graph_->Stop();
    }
    if (lio_) {
        lio_->Stop();
    }
    running_ = false;
    if (g2p5_) {
        g2p5_->Quit();
    }
}

}  // namespace lightning
