#ifndef LIGHTNING_CORE_LIO_AA_FASTERLIO_ZERO_VELOCITY_UPDATER_H_
#define LIGHTNING_CORE_LIO_AA_FASTERLIO_ZERO_VELOCITY_UPDATER_H_

#include "common/measure_group.h"
#include "core/lio/aa-fasterlio/eskf.hpp"

namespace lightning {

// Uses new wheel/IMU samples to confirm a stationary scan and update its state.
// 轮速仅参与静止判定；更新使用零速度、零角速度和重力平衡观测。
// IMU 测量在真实 IMU 系表达，NavState 中的速度仍在世界系表达。
class ZeroVelocityUpdater {
 public:
  struct Options {
    double confirm_duration = 0.3;          // Seconds of continuous quiet input.
    double max_wheel_speed = 0.02;          // m/s, at the wheel center.
    double max_wheel_angular_speed = 0.02;  // rad/s, from wheel encoders.
    double max_imu_angular_speed = 0.02;    // rad/s, bias-corrected gyro RMS.
    double imu_noise_multiplier = 10.0;    // Scales IMU observation variances.
    double velocity_variance = 0.0001;     // (m/s)^2, once per scan.
  };

  void SetOptions(const Options& options);
  // 清除静止确认、日志标志和批次统计的有效性，不修改主滤波器。
  void Reset();

  // Commits only a successful static update; otherwise leaves kf_state and
  // measurements unchanged so the caller can run ordinary motion processing.
  // 处理区间为主滤波器当前时间到扫描结束；确认时间不足也返回 false。
  bool TryUpdate(const MeasureGroup& measures,
                 const ESKF::ProcessNoiseType& process_noise, ESKF& kf_state);

  // Called only during TryUpdate; relinearizes the same fixed batch statistics.
  // 每次 ESKF 迭代按当前状态重新线性化，速度约束每帧只加入一次。
  void ObsModel(NavState& state,
                ESKF::CustomObservationModel& observation) const;

 private:
  // 由本批统计量和当前状态构造 H^T R^-1 H、H^T R^-1 r，
  // 返回包含样本波动能量的 sum(r^T R^-1 r)。
  double BuildImuObservation(
      const NavState& state, ESKF::CustomObservationModel& observation) const;

  Options options_;
  // 连续通过轮速和 IMU 静止判据的起点；负值表示尚未开始确认。
  double stationary_start_time_ = -1.0;
  // 仅用于在进入、退出静止更新时各打印一次日志。
  bool zupt_active_ = false;
  // 统计量仅在本次 TryUpdate 内有效，返回前将数量清零。
  int imu_sample_count_ = 0;
  Vec6d mean_imu_measurement_ = Vec6d::Zero();  // [角速度均值; 加速度均值]。
  // sum((measurement - mean)^T * W * (measurement - mean))。
  double imu_scatter_energy_ = 0.0;
  // 各轴 IMU 观测方差的倒数，本次检测及 ESKF 迭代共用。
  Vec3d gyro_information_ = Vec3d::Zero();
  Vec3d accel_information_ = Vec3d::Zero();
};

}  // namespace lightning

#endif  // LIGHTNING_CORE_LIO_AA_FASTERLIO_ZERO_VELOCITY_UPDATER_H_

