#pragma once

#include <map>
#include <utility>

#include "common/imu.h"
#include "common/sensor_data.h"
#include "core/lio/laser_mapping.h"
#include "core/localization/localization_result.h"
#include "core/system/async_message_process.h"

/// 预声明
namespace lightning {

namespace loc {

class LidarLoc;
class PGO;

/**
 * 实时定位接口实现
 */
class Localization {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = false;  // 在线模式还是离线模式
        bool with_ui_ = false;      // 是否请求外层UI消费者

        /// 参数
        SE3 T_body_lidar_;

        bool enable_lidar_odom_skip_ = false;  // 是否允许激光里程计跳帧
        int lidar_odom_skip_num_ = 1;          // 如果允许跳帧，跳多少帧
        bool enable_lidar_loc_skip_ = true;    // 是否允许激光定位跳帧
        bool enable_lidar_loc_rviz_ = false;   // 是否允许调试用rviz
        int lidar_loc_skip_num_ = 4;           // 如果允许跳帧，跳多少帧
        bool loc_on_kf_ = false;
    };

    Localization(Options options = Options());
    ~Localization() = default;

    /**
     * 初始化，读配置参数
     * @param yaml_path
     * @param global_map_path
     * @param init_reloc_pose
     */
    bool Init(const std::string& yaml_path, const std::string& global_map_path);

    /// Process a transport-independent lidar scan.
    bool ProcessLidar(const TimedPointCloudData& scan);
    bool ProcessLidar(CloudPtr scan);

    /// Process an internal IMU sample.
    void ProcessIMU(const IMUPtr& imu);

    // void ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) override;

    /// 由外部设置pose，适用于手动重定位
    void SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    /// TODO: 其他初始化逻辑

    /// TODO: 处理odom消息

    /// 结束，保存临时地图
    void Finish();

    /// 异步处理函数
    void LidarOdomProcCloud(CloudPtr);
    void LidarLocProcCloud(CloudPtr);

    using ResultCallback = std::function<void(const LocalizationResult& result)>;
    using LocStateCallback = std::function<void(LocalizationStatus status)>;
    using NavStateCallback = std::function<void(const NavState&)>;
    using RecentPoseCallback = std::function<void(const SE3&)>;
    using ScanCallback = std::function<void(const CloudPtr&, const SE3&)>;
    using MapUpdateCallback =
        std::function<void(const std::map<int, CloudPtr>&, const std::map<int, CloudPtr>&)>;

    void SetResultCallback(ResultCallback callback) { result_callback_ = std::move(callback); }
    void SetLocStateCallback(LocStateCallback callback) { loc_state_callback_ = std::move(callback); }
    void SetNavStateCallback(NavStateCallback callback) {
        nav_state_callback_ = std::move(callback);
    }
    void SetRecentPoseCallback(RecentPoseCallback callback) {
        recent_pose_callback_ = std::move(callback);
    }
    void SetScanCallback(ScanCallback callback) { scan_callback_ = std::move(callback); }
    void SetMapUpdateCallback(MapUpdateCallback callback) {
        map_update_callback_ = std::move(callback);
    }

    // void SetPathCallback(std::function<void(const nav_msgs::msg::Path& path)>&& callback);
   private:
    /// 模块  ========================================================================================================
    std::mutex global_mutex_;  // 防止处理过程中被重复init
    Options options_;

    /// 预处理
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess

    /// 前端
    std::shared_ptr<LaserMapping> lio_ = nullptr;
    Keyframe::Ptr lio_kf_ = nullptr;

    // pose graph
    std::shared_ptr<PGO> pgo_ = nullptr;

    // lidar localization
    std::shared_ptr<LidarLoc> lidar_loc_;

    /// TODO async 处理
    sys::AsyncMessageProcess<CloudPtr> lidar_odom_proc_cloud_;  // lidar odom 处理点云
    sys::AsyncMessageProcess<CloudPtr> lidar_loc_proc_cloud_;   // lidar loc 处理点云

    /// 结果数据 =====================================================================================================
    LocalizationResult loc_result_;

    /// 框架相关
    ResultCallback result_callback_;
    LocStateCallback loc_state_callback_;
    NavStateCallback nav_state_callback_;
    RecentPoseCallback recent_pose_callback_;
    ScanCallback scan_callback_;
    MapUpdateCallback map_update_callback_;

    /// 输入检查
    double last_imu_time_ = 0;
    double last_odom_time_ = 0;
    double last_cloud_time_ = 0;
};
}  // namespace loc

}  // namespace lightning
