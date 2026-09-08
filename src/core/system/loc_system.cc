//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"

#include "core/localization/localization.h"
#include "io/yaml_io.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    sys::SensorDispatcher::Options dispatcher_options;
    sensor_dispatcher_ = std::make_unique<sys::SensorDispatcher>(
        dispatcher_options,
        [this](const IMUPtr& imu) { ProcessIMUOnWorker(imu); },
        [this](const TimedPointCloudData& cloud) { ProcessLidarOnWorker(cloud); });
}

LocSystem::~LocSystem() { Stop(); }

bool LocSystem::Init(const std::string& yaml_path) {
    loc::Localization::Options options;
    options.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(options);

    YAML_IO yaml(yaml_path);
    const std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    if (result_callback_) {
        loc_->SetResultCallback(result_callback_);
    }
    if (nav_state_callback_) {
        loc_->SetNavStateCallback(nav_state_callback_);
    }
    if (recent_pose_callback_) {
        loc_->SetRecentPoseCallback(recent_pose_callback_);
    }
    if (scan_callback_) {
        loc_->SetScanCallback(scan_callback_);
    }
    if (map_update_callback_) {
        loc_->SetMapUpdateCallback(map_update_callback_);
    }

    if (!loc_->Init(yaml_path, map_path)) {
        LOG(ERROR) << "failed to initialize localization core";
        loc_.reset();
        return false;
    }

    map_loaded_ = true;
    LOG(INFO) << "localization core has been created.";
    return true;
}

void LocSystem::SetInitPose(const SE3& pose) {
    if (!loc_ || !map_loaded_) {
        return;
    }

    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();
    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
    if (sensor_dispatcher_) {
        sensor_dispatcher_->Start();
    }
}

void LocSystem::ProcessIMU(const IMUPtr& imu) {
    if (!loc_started_ || !loc_ || !imu) {
        return;
    }
    if (sensor_dispatcher_) {
        const InputResult result = sensor_dispatcher_->AddImu(imu);
        if (result != InputResult::Accepted && result != InputResult::NotRunning) {
            LOG(WARNING) << "reject IMU input: " << static_cast<int>(result);
        }
        return;
    }
    ProcessIMUOnWorker(imu);
}

bool LocSystem::ProcessLidar(const TimedPointCloudData& cloud) {
    if (!loc_started_ || !loc_ || cloud.points.empty()) {
        return false;
    }
    if (sensor_dispatcher_) {
        auto scan = std::make_shared<TimedPointCloudData>(cloud);
        const InputResult result = sensor_dispatcher_->AddPointCloud(std::move(scan));
        if (result != InputResult::Accepted && result != InputResult::NotRunning) {
            LOG(WARNING) << "reject lidar input: " << static_cast<int>(result);
        }
        return result == InputResult::Accepted;
    }
    return ProcessLidarOnWorker(cloud);
}

bool LocSystem::ProcessLidar(CloudPtr cloud) {
    if (!loc_started_ || !loc_) {
        return false;
    }
    return loc_->ProcessLidar(std::move(cloud));
}

void LocSystem::ProcessIMUOnWorker(const IMUPtr& imu) {
    if (loc_ && loc_started_ && imu) {
        loc_->ProcessIMU(imu);
    }
}

bool LocSystem::ProcessLidarOnWorker(const TimedPointCloudData& cloud) {
    return loc_ && loc_started_ && loc_->ProcessLidar(cloud);
}

void LocSystem::SetResultCallback(ResultCallback callback) {
    result_callback_ = std::move(callback);
    if (loc_) {
        loc_->SetResultCallback(result_callback_);
    }
}

void LocSystem::SetNavStateCallback(NavStateCallback callback) {
    nav_state_callback_ = std::move(callback);
    if (loc_) {
        loc_->SetNavStateCallback(nav_state_callback_);
    }
}

void LocSystem::SetRecentPoseCallback(RecentPoseCallback callback) {
    recent_pose_callback_ = std::move(callback);
    if (loc_) {
        loc_->SetRecentPoseCallback(recent_pose_callback_);
    }
}

void LocSystem::SetScanCallback(ScanCallback callback) {
    scan_callback_ = std::move(callback);
    if (loc_) {
        loc_->SetScanCallback(scan_callback_);
    }
}

void LocSystem::SetMapUpdateCallback(MapUpdateCallback callback) {
    map_update_callback_ = std::move(callback);
    if (loc_) {
        loc_->SetMapUpdateCallback(map_update_callback_);
    }
}

sys::SensorDispatcher::Stats LocSystem::GetInputStats() const {
    return sensor_dispatcher_ ? sensor_dispatcher_->GetStats() : sys::SensorDispatcher::Stats();
}

void LocSystem::Stop() {
    if (sensor_dispatcher_) {
        sensor_dispatcher_->Stop(sys::SensorDispatcher::StopMode::kDrain);
    }

    loc_started_ = false;
    if (!loc_) {
        return;
    }

    loc_->Finish();
    loc_.reset();
    map_loaded_ = false;
}

}  // namespace lightning
