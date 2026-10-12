#ifndef LIGHTNING_AA_FASTERLIO_LASER_MAPPING_H
#define LIGHTNING_AA_FASTERLIO_LASER_MAPPING_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/options.h"
#include "common/std_types.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/aa-fasterlio/eskf.hpp"
#include "core/lio/aa-fasterlio/imu_processing.hpp"
#include "core/lio/keyframe_selector.h"
#include "core/lio/aa-fasterlio/pointcloud_preprocess.h"
#include "core/lio/lio.h"
#include "core/lio/scan_accumulator.h"

namespace lightning {

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping final : public LIO {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        bool enable_icp_part_ = true;    // 是否添加ICP部分
        double plane_icp_weight_ = 1.0;  // 点面ICP部分的权重
        double icp_weight_ = 100;        // ICP部分的权重

        int min_pts = 300;  // 配准所需的点数

        /// 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;

        bool proj_kfs_ = false;
        int max_proj_kfs_ = 5;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() override;

    /// init without ros
    bool Init(const std::string& config_yaml);
    bool Init(const std::string& config_yaml, const LIOOptions& options) override;

    LIOCapabilities GetCapabilities() const override { return {true, false}; }

    void SetResultCallback(ResultCallback callback) override {
        result_callback_ = std::move(callback);
    }
    void SetPredictionCallback(PredictionCallback callback) override {
        prediction_callback_ = std::move(callback);
    }
    void SetDataCallback(DataCallback callback) override {
        lio_data_callback_ = std::move(callback);
    }

    LIOInputStatus AddImu(const IMUPtr& imu) override;
    LIOInputStatus AddPointCloud(const TimedPointCloudData& scan) override;
    LIOInputStatus AddWheelOdometry(const WheelOdometryData& data) override;
    void Stop() override;

   private:
    bool Run();
    bool HasPendingLidar() const { return lidar_pushed_ || !lidar_buffer_.empty(); }

    // These methods are implementation details behind the transport-neutral
    // AddImu() and AddPointCloud() entry points.
    bool ProcessPointCloud(const TimedPointCloudData& scan);
    void ProcessIMU(const IMUPtr& imu);

    // sync lidar with imu
    bool SyncPackages();

    void ObsModel(NavState& s, ESKF::CustomObservationModel& obs);

    inline void PointBodyToWorld(const PointType& pi, PointType& po) {
        Vec3d p_global(state_point_.rot_ *
                           (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    void NotifyLIOData(const CloudPtr& cloud, const SE3& pose, double timestamp);

    bool LoadParamsFromYAML(const std::string &yaml);

    /// Select a frontend-local keyframe without allocating a backend ID.
    bool SelectKeyframe();

    void NotifyPrediction(const NavState& state);
    void NotifyResult(const NavState& state, LIOUpdateType update_type, bool keyframe_selected);

    Options options_;

    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process
    DataCallback lio_data_callback_;
    ResultCallback result_callback_;
    PredictionCallback prediction_callback_;

    /// local map related
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();
    std::string map_file_path_;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion, in the LiDAR frame
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan, in the LiDAR frame
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan, in the local frame
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    /// 点面相关
    std::vector<PointVector> nearest_points_;  // nearest points of current scan
    std::vector<Vec4f> corr_pts_;              // inlier pts
    std::vector<Vec4f> corr_norm_;             // inlier plane norms
    std::vector<float> residuals_;             // point-to-plane residuals
    std::vector<char> point_selected_surf_;    // selected points
    std::vector<Vec4f> plane_coef_;            // plane coeffs

    /// 点到点相关
    std::vector<char> point_selected_icp_;  // 点到点的selected points

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    /// options
    bool keep_first_imu_estimation_ = false;  // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_surf_ = 0, frame_num_ = 0, effect_feat_icp_ = 0;

    double last_lidar_time_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    bool use_aa_ = false;  // use anderson acceleration?

    KeyframeSelector keyframe_selector_;
    ScanAccumulator scan_accumulator_;

    std::atomic_bool running_ = false;
    bool last_keyframe_selected_ = false;

};

}  // namespace lightning

#endif  // LIGHTNING_AA_FASTERLIO_LASER_MAPPING_H
