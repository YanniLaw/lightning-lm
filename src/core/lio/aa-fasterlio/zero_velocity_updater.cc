#include "core/lio/aa-fasterlio/zero_velocity_updater.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <utility>

#include <Eigen/Cholesky>
#include <boost/math/distributions/chi_squared.hpp>
#include <glog/logging.h>

namespace lightning {

void ZeroVelocityUpdater::SetOptions(const Options& options) {
  options_ = options;
  Reset();
}

void ZeroVelocityUpdater::Reset() {
  LOG_IF(INFO, zupt_active_) << "[ZUPT] exited";
  zupt_active_ = false;
  stationary_start_time_ = -1.0;
  imu_sample_count_ = 0;
}

double ZeroVelocityUpdater::BuildImuObservation(
    const NavState& state, ESKF::CustomObservationModel& observation) const {
  observation.HTH_.setZero();
  observation.HTr_.setZero();
  if (imu_sample_count_ == 0) return 0.0;

  // 静止时 a_m = ba - R^T g；按当前姿态投影重力，保留 IMU 安装倾角。
  const Vec3d gravity_in_imu = state.rot_.inverse() * state.grav_;
  Eigen::Matrix<double, 6, ESKF::state_dim_> jacobian =
      Eigen::Matrix<double, 6, ESKF::state_dim_>::Zero();
  // h = [w_m - bg; a_m - ba + R^T g]，残差 r = -h。
  // H 是 h 的雅可比；按当前右扰动定义，姿态块为 hat(R^T g)。
  jacobian.block<3, 3>(0, NavState::kBgIdx) = -Mat3d::Identity();
  jacobian.block<3, 3>(3, NavState::kRotIdx) = SO3::hat(gravity_in_imu);
#if LIGHTNING_ESTIMATE_ACCEL_BIAS
  jacobian.block<3, 3>(3, NavState::kBaIdx) = -Mat3d::Identity();
#endif
  Vec6d information;
  information << gyro_information_, accel_information_;
  const auto weighted_jacobian =
      (jacobian.transpose() * information.asDiagonal()).eval();
  Vec6d mean_residual;
  mean_residual << state.bg_, state.Getba() - gravity_in_imu;
  mean_residual -= mean_imu_measurement_;
  // 同一状态下各样本的 H、W 相同，批次形式与逐样本累加等价。
  observation.HTH_.noalias() =
      imu_sample_count_ * weighted_jacobian * jacobian;
  observation.HTr_.noalias() =
      imu_sample_count_ * weighted_jacobian * mean_residual;
  // 保留均值之外的波动能量，完整批次 NIS 才不会退化为均值检验。
  return imu_sample_count_ *
             mean_residual.dot(information.cwiseProduct(mean_residual)) +
         imu_scatter_energy_;
}

void ZeroVelocityUpdater::ObsModel(
    NavState& state, ESKF::CustomObservationModel& observation) const {
  BuildImuObservation(state, observation);
  observation.valid_ = imu_sample_count_ > 0;
  if (!observation.valid_) return;

  // A single velocity constraint per scan, independent of its IMU count.
  // v_W = 0 是世界系速度观测，避免随 IMU 数量重复加权。
  const double velocity_information = 1.0 / options_.velocity_variance;
  observation.HTH_.block<3, 3>(NavState::kVelIdx, NavState::kVelIdx)
      .diagonal().array() += velocity_information;
  observation.HTr_.segment<3>(NavState::kVelIdx) -=
      velocity_information * state.vel_;
}

bool ZeroVelocityUpdater::TryUpdate(
    const MeasureGroup& measures, const ESKF::ProcessNoiseType& process_noise,
    ESKF& kf_state) {
  // 以主滤波器时间为左端，静止确认与协方差演化使用同一时间范围。
  const double begin_time = kf_state.GetX().timestamp_;
  const double end_time = measures.lidar_end_time_;
  if (end_time <= begin_time || measures.imu_.size() < 2) {
    Reset();
    return false;
  }

  // 本区间所有新轮速均须足够小，角速度用于排除原地转动。
  // 轮心速度虽在 IMU 轴下表达，其模长不受安装倾角影响。
  int wheel_count = 0;
  for (const auto& odom : measures.odom_) {
    if (odom->timestamp <= begin_time || odom->timestamp > end_time) continue;
    if (odom->linear_velocity.norm() > options_.max_wheel_speed ||
        odom->angular_velocity.norm() > options_.max_wheel_angular_speed) {
      Reset();
      return false;
    }
    ++wheel_count;
  }
  if (wheel_count < 2) {
    Reset();
    return false;
  }

  // Preserve the existing per-IMU dt^2 driving-noise convention.
  // 静止模型只推进 bg 和可选 ba 的随机游走协方差，不进行普通运动积分。
  // 累加各个 IMU 小区间的 dt^2，并补齐最后一条 IMU 到扫描结束的区间。
  imu_sample_count_ = 0;
  mean_imu_measurement_.setZero();
  Vec6d centered_sum_squares = Vec6d::Zero();
  double cursor_time = begin_time;
  double squared_duration = 0.0;
  for (const auto& imu : measures.imu_) {
    // 只统计 (begin_time, end_time]，保持原有观测样本的筛选规则。
    if (imu->timestamp <= begin_time || imu->timestamp > end_time) continue;
    Vec6d measurement;
    measurement << imu->angular_velocity, imu->linear_acceleration;
    const Vec6d deviation = measurement - mean_imu_measurement_;
    ++imu_sample_count_;
    mean_imu_measurement_ += deviation / imu_sample_count_;
    // 在线累计中心化平方差，避免从较大的原始二阶矩中相减。
    centered_sum_squares +=
        deviation.cwiseProduct(measurement - mean_imu_measurement_);

    if (imu->timestamp <= cursor_time) continue;
    squared_duration += std::pow(imu->timestamp - cursor_time, 2);
    cursor_time = imu->timestamp;
  }
  // sum(||w - bg||^2) = N * ||mean(w) - bg||^2 + sum(gyro_scatter)。
  // 使用更新前的零偏和三轴 RMS，避免正反转动在均值中抵消；比较平方免开方。
  const double squared_angular_speed_sum =
      imu_sample_count_ *
          (mean_imu_measurement_.head<3>() - kf_state.GetX().bg_).squaredNorm() +
      centered_sum_squares.head<3>().sum();
  if (squared_angular_speed_sum >
      imu_sample_count_ * options_.max_imu_angular_speed *
          options_.max_imu_angular_speed) {
    Reset();
    return false;
  }

  squared_duration += std::pow(end_time - cursor_time, 2);
  ESKF::CovType covariance = kf_state.GetP();
  covariance.block<3, 3>(NavState::kBgIdx, NavState::kBgIdx).diagonal() +=
      squared_duration * process_noise.block<3, 3>(6, 6).diagonal();
#if LIGHTNING_ESTIMATE_ACCEL_BIAS
  covariance.block<3, 3>(NavState::kBaIdx, NavState::kBaIdx).diagonal() +=
      squared_duration * process_noise.block<3, 3>(9, 9).diagonal();
#endif

  // 本次检测和迭代更新共用固定的批次统计量与各轴观测权重。
  // Q 的前两个三维块按现有工程的有效离散方差使用。
  gyro_information_ =
      (options_.imu_noise_multiplier *
       process_noise.block<3, 3>(0, 0).diagonal()).cwiseInverse();
  accel_information_ =
      (options_.imu_noise_multiplier *
       process_noise.block<3, 3>(3, 3).diagonal()).cwiseInverse();
  Vec6d imu_information;
  imu_information << gyro_information_, accel_information_;
  imu_scatter_energy_ = imu_information.dot(centered_sum_squares);
  ESKF::CustomObservationModel observation;
  // 检测仅使用 IMU 静止模型，避免估计速度漂移阻止真实停车通过检验。
  const double squared_error =
      BuildImuObservation(kf_state.GetX(), observation);

  Eigen::LDLT<ESKF::CovType> prior_solver(covariance);
  if (imu_sample_count_ == 0 || prior_solver.info() != Eigen::Success ||
      !prior_solver.isPositive()) {
    LOG_EVERY_N(WARNING, 100)
        << "[ZUPT] skipped: no new IMU samples or invalid prior covariance, "
        << "IMU samples=" << imu_sample_count_;
    Reset();
    return false;
  }
  // Woodbury form of r^T (H P H^T + R)^-1 r; only a 12/15D solve.
  // 用前面累加的 A、b、c 计算 NIS = c - b^T(P^-1 + A)^-1 b，
  // 避免构造随 IMU 数量增长的大型创新协方差矩阵。
  const ESKF::CovType information =
      prior_solver.solve(ESKF::CovType::Identity()) + observation.HTH_;
  Eigen::LDLT<ESKF::CovType> innovation_solver(information);
  if (innovation_solver.info() != Eigen::Success ||
      !innovation_solver.isPositive()) {
    LOG_EVERY_N(WARNING, 100)
        << "[ZUPT] skipped: invalid innovation information matrix";
    Reset();
    return false;
  }
  const double nis =
      squared_error - observation.HTr_.dot(
          innovation_solver.solve(observation.HTr_));
  // 每条 IMU 含三维角速度和三维加速度残差，自由度仍为 6N。
  const double threshold = boost::math::quantile(
      boost::math::chi_squared(6 * imu_sample_count_), 0.95);
  // 理论 NIS 非负，仅容许浮点消减造成的舍入量级负值。
  if (!std::isfinite(nis) ||
      nis < -1e-10 * std::max(1.0, squared_error) ||
      nis > threshold) {
    Reset();
    return false;
  }

  // 轮速和 IMU 同时通过后开始连续计时；确认期间由调用方继续正常融合。
  // 任一判据失败会通过前面的 Reset() 清除计时。
  if (stationary_start_time_ < 0.0) {
    stationary_start_time_ = begin_time;
  }
  if (end_time - stationary_start_time_ < options_.confirm_duration) {
    imu_sample_count_ = 0;
    return false;
  }

  // Static propagation and observation are committed together, exactly once.
  // 在副本上推进协方差、时间并更新；被拒绝时主状态、协方差和时间均不变。
  ESKF candidate = kf_state;
  candidate.ChangeP(covariance);
  candidate.SetTime(end_time);
  if (!candidate.Update(ESKF::ObsType::ZERO_VELOCITY, 1.0)) {
    LOG_EVERY_N(WARNING, 100)
        << "[ZUPT] update rejected by ESKF, continuing normal fusion";
    Reset();
    return false;
  }
  LOG_IF(INFO, !zupt_active_)
      << "[ZUPT] entered: speed=" << kf_state.GetX().vel_.norm() << " -> "
      << candidate.GetX().vel_.norm() << " m/s, t=" << std::fixed
      << std::setprecision(4) << end_time;
  zupt_active_ = true;
  // 成功后提交完整后验；调用方据此跳过本帧普通轮速/NHC 和雷达更新。
  kf_state = std::move(candidate);
  imu_sample_count_ = 0;
  return true;
}

}  // namespace lightning

