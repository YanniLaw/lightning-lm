//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/sensor_data.h"
#include "core/localization/localization_result.h"
#include "core/system/sensor_dispatcher.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// Process a transport-independent lidar scan.
    bool ProcessLidar(const TimedPointCloudData& cloud);
    bool ProcessLidar(CloudPtr cloud);

    using ResultCallback = std::function<void(const loc::LocalizationResult&)>;
    using NavStateCallback = std::function<void(const NavState&)>;
    using RecentPoseCallback = std::function<void(const SE3&)>;
    using ScanCallback = std::function<void(const CloudPtr&, const SE3&)>;
    using MapUpdateCallback =
        std::function<void(const std::map<int, CloudPtr>&, const std::map<int, CloudPtr>&)>;
    void SetResultCallback(ResultCallback callback);
    void SetNavStateCallback(NavStateCallback callback);
    void SetRecentPoseCallback(RecentPoseCallback callback);
    void SetScanCallback(ScanCallback callback);
    void SetMapUpdateCallback(MapUpdateCallback callback);

    sys::SensorDispatcher::Stats GetInputStats() const;

    /// Stop processing.  Repeated calls are safe.
    void Stop();

   private:
    void ProcessIMUOnWorker(const IMUPtr& imu);
    bool ProcessLidarOnWorker(const TimedPointCloudData& cloud);

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 外层运行时控制和结果回调
    ResultCallback result_callback_;
    NavStateCallback nav_state_callback_;
    RecentPoseCallback recent_pose_callback_;
    ScanCallback scan_callback_;
    MapUpdateCallback map_update_callback_;
    std::unique_ptr<sys::SensorDispatcher> sensor_dispatcher_;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
