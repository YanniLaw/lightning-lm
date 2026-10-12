#include "core/localization/localization.h"

#include <utility>

#include <yaml-cpp/yaml.h>

#include "core/lio/lio_factory.h"
#include "core/lio/lio_result_conversion.h"
#include "core/localization/lidar_loc/lidar_loc.h"
#include "core/localization/pose_graph/pgo.h"
#include "common/std_types.h"
#include "io/yaml_io.h"

namespace lightning::loc {

// ！ 构造函数
Localization::Localization(Options options) { options_ = options; }

// ！初始化函数
bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (lidar_loc_ != nullptr) {
        // 若已经启动，则变为初始化
        Finish();
    }

    YAML_IO yaml(yaml_path);
    options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");

    std::string lio_type = "aa_fasterlio";
    const YAML::Node yaml_node = YAML::LoadFile(yaml_path);
    if (yaml_node["system"] && yaml_node["system"]["lio_type"]) {
        lio_type = yaml_node["system"]["lio_type"].as<std::string>();
    }

    lio_ = CreateLIO(lio_type);
    if (!lio_) {
        LOG(ERROR) << "failed to create LIO frontend: " << lio_type;
        return false;
    }
    if (!lio_->GetCapabilities().imu_prediction) {
        LOG(ERROR) << "LIO frontend does not provide IMU prediction required by localization";
        lio_.reset();
        return false;
    }

    lio_->SetResultCallback([this](const LIOResult& result) { HandleLIOResult(result); });
    lio_->SetPredictionCallback([this](const LIOState& state) { HandleLIOPrediction(state); });

    LIOOptions lio_options;
    lio_options.usage = LIOUsage::kLocalization;
    if (!lio_->Init(yaml_path, lio_options)) {
        LOG(ERROR) << "failed to initialize LIO frontend";
        lio_.reset();
        return false;
    }

    /// 激光定位
    LidarLoc::Options lidar_loc_options;
    lidar_loc_options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
    lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
    lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
    lidar_loc_options.map_option_.map_path_ = global_map_path;
    lidar_loc_ = std::make_shared<LidarLoc>(lidar_loc_options);

    if (map_update_callback_) {
        lidar_loc_->SetMapUpdateCallback(map_update_callback_);
    }

    lidar_loc_->Init(yaml_path);

    /// pose graph
    pgo_ = std::make_shared<PGO>();
    pgo_->SetDebug(false);

    ///  各模块的异步调用
    options_.enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
    options_.enable_lidar_loc_rviz_ = yaml.GetValue<bool>("system", "enable_lidar_loc_rviz");
    options_.lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");
    options_.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
    options_.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
    options_.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");

    lidar_loc_proc_cloud_.SetMaxSize(1);

    lidar_loc_proc_cloud_.SetName("激光定位");

    // 允许跳帧
    lidar_loc_proc_cloud_.SetSkipParam(options_.enable_lidar_loc_skip_, options_.lidar_loc_skip_num_);

    lidar_loc_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarLocProcCloud(cloud); });

    if (options_.online_mode_) {
        lidar_loc_proc_cloud_.Start();
    }

    /// TODO: 发布
    pgo_->SetHighFrequencyGlobalOutputHandleFunction([this](const LocalizationResult& res) {
        // if (loc_result_.timestamp_ > 0) {
        //             double loc_fps = 1.0 / (res.timestamp_ - loc_result_.timestamp_);
        //             // LOG_EVERY_N(INFO, 10) << "loc fps: " << loc_fps;
        //         }

        loc_result_ = res;

        if (result_callback_) {
            result_callback_(loc_result_);
        }

        if (nav_state_callback_) {
            nav_state_callback_(loc_result_.ToNavState());
        }
        if (recent_pose_callback_) {
            recent_pose_callback_(loc_result_.pose_);
        }
    });

    return true;
}

bool Localization::ProcessLidar(const TimedPointCloudData& scan) {
    if (scan.timestamp_ns <= 0 || scan.points.empty()) {
        return false;
    }

    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return false;
    }

    const LIOInputStatus status = lio_->AddPointCloud(scan);
    if (status != LIOInputStatus::kAccepted) {
        LOG(WARNING) << "reject lidar input in localization: " << static_cast<int>(status);
        return false;
    }
    return true;
}

void Localization::HandleLIOPrediction(const LIOState& state) {
    NavState nav_state;
    if (!ConvertToNavState(state, nav_state) || !lidar_loc_ || !pgo_) {
        return;
    }

    lidar_loc_->ProcessDR(nav_state);
    pgo_->ProcessDR(nav_state);
}

void Localization::HandleLIOResult(const LIOResult& result) {
    if (result.update_type != LIOUpdateType::kScanMatched) {
        return;
    }

    NavState lidar_odom_state;
    if (!ConvertToNavState(result.state, lidar_odom_state) || !lidar_loc_ || !pgo_) {
        return;
    }

    lidar_loc_->ProcessLO(lidar_odom_state);
    pgo_->ProcessLidarOdom(lidar_odom_state);

    if (options_.loc_on_kf_ && !result.keyframe_selected) {
        return;
    }

    const auto& projected_cloud = result.projected_cloud ? result.projected_cloud : result.cloud;
    if (!projected_cloud || projected_cloud->empty()) {
        LOG(WARNING) << "LIO result does not contain a point cloud for localization";
        return;
    }

    auto scan = std::make_shared<PointCloudType>(*projected_cloud);
    if (options_.online_mode_) {
        lidar_loc_proc_cloud_.AddMessage(std::move(scan));
    } else {
        LidarLocProcCloud(std::move(scan));
    }
}

void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    lidar_loc_->ProcessCloud(scan_undist);

    auto res = lidar_loc_->GetLocalizationResult();
    pgo_->ProcessLidarLoc(res);

    if (scan_callback_) {
        // Twi with Til, here pose means Twl, thus Til=I.
        scan_callback_(scan_undist, res.pose_);
    }

    if (loc_state_callback_) {
        LOG(INFO) << "loc_state: " << static_cast<int>(res.status_);
        loc_state_callback_(res.status_);
    }

    // cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
    // cv::imshow("img", img);
    // cv::waitKey(0);
}

void Localization::ProcessIMU(const IMUPtr& imu) {
    if (!imu) {
        return;
    }

    UL lock(global_mutex_);

    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    const LIOInputStatus status = lio_->AddImu(imu);
    if (status != LIOInputStatus::kAccepted) {
        LOG(WARNING) << "reject IMU input in localization: " << static_cast<int>(status);
    }
}

// void Localization::ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
//     UL lock(global_mutex_);
//
//     if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
//         return;
//     }
//     double this_odom_time = ToSec(odom_msg->header.stamp);
//     if (last_odom_time_ > 0 && this_odom_time < last_odom_time_) {
//         LOG(WARNING) << "Odom Time Abnormal:" << this_odom_time << ", last: " << last_odom_time_;
//     }
//     last_odom_time_ = this_odom_time;
//
//     lio_->ProcessOdometry(odom_msg);
//
//     if (!lio_->GetbOdomHF()) {
//         return;
//     }
//
//     auto dr_state = lio_->GetStateHF(mapping::FasterLioMapping::kHFStateOdomFiltered);
//
//     constexpr auto kThVbrbStill = 0.03;  // 0.08;
//     constexpr auto kThOmegaStill = 0.03;
//     if (dr_state.Getvwi().norm() < kThVbrbStill && dr_state.Getwii().norm() < kThOmegaStill) {
//         dr_state.is_parking_ = true;
//         dr_state.Setvwi(Vec3d::Zero());
//         dr_state.Setwii(Vec3d::Zero());
//     }
//
//     lidar_loc_->ProcessDR(dr_state);
//     pgo_->ProcessDR(dr_state);
// }

void Localization::Finish() {
    // Stop producers before releasing the map they access.  This keeps the
    // localization worker threads from racing with map teardown.
    lidar_loc_proc_cloud_.Quit();
    if (lio_) {
        lio_->Stop();
    }
    if (lidar_loc_) {
        lidar_loc_->Finish();
    }

    pgo_.reset();
    lidar_loc_.reset();
    lio_.reset();
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    /// 设置外部重定位的pose
    if (lidar_loc_) {
        lidar_loc_->SetInitialPose(SE3(q, t));
    }
}

}  // namespace lightning::loc
