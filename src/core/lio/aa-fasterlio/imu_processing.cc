#include "core/lio/aa-fasterlio/imu_processing.hpp"

#include <utility>

#include "core/lio/aa-fasterlio/deskew_trajectory.h"

namespace lightning {

// 对单条 IMU 按需滤波，并将加速度统一为 m/s²；处理结果供各条预测路径复用。
IMU ImuProcess::PreprocessImu(const IMU& raw) {
  IMU prepared = use_imu_filter_ ? filter_.Filter(raw) : raw;
  prepared.linear_acceleration *= acc_scale_factor_;
  return prepared;
}

// imu_from_wheel 为轮心系到真实 IMU 系的 T_IB。
// 三轴观测方差依次对应前向轮速、横向 NHC 和竖向 NHC，取倒数得到观测权重。
void ImuProcess::ConfigureWheelSpeedObservation(
    const SE3& imu_from_wheel, const Vec3d& velocity_variance) {
  imu_from_wheel_ = imu_from_wheel;
  wheel_information_ = velocity_variance.cwiseInverse().asDiagonal();
}

// 构造轮心坐标系下的三维速度观测，供 ESKF 的 WHEEL_SPEED 更新迭代调用。
// 本函数只计算残差和雅可比，不取新测量，也不推进滤波状态。
void ImuProcess::WheelSpeedObsModel(
    NavState& state, ESKF::CustomObservationModel& observation) {
  const Mat3d wheel_from_imu =
      imu_from_wheel_.rotationMatrix().transpose();
  const Vec3d& lever_arm = imu_from_wheel_.translation();
  // 先将世界系速度转到 IMU 系，再用去零偏角速度补偿 IMU 原点到轮心的杆臂速度。
  const Vec3d velocity_in_imu = state.rot_.inverse() * state.vel_;
  const Vec3d predicted =
      wheel_from_imu *
      (velocity_in_imu + (wheel_gyro_ - state.bg_).cross(lever_arm));
  // 编码器提供前向速度，NHC 将轮心横向和竖向速度作为零值软约束。
  const Vec3d residual = Vec3d(wheel_forward_speed_, 0.0, 0.0) - predicted;

  // 按 NavState 的右扰动定义，填写姿态、世界系速度和陀螺零偏的雅可比。
  Eigen::Matrix<double, 3, ESKF::state_dim_> jacobian =
      Eigen::Matrix<double, 3, ESKF::state_dim_>::Zero();
  jacobian.block<3, 3>(0, NavState::kRotIdx) =
      wheel_from_imu * SO3::hat(velocity_in_imu);
  jacobian.block<3, 3>(0, NavState::kVelIdx) =
      wheel_from_imu * state.rot_.matrix().transpose();
  jacobian.block<3, 3>(0, NavState::kBgIdx) =
      wheel_from_imu * SO3::hat(lever_arm);
  // 提交 ESKF 使用的 HᵀR⁻¹H 和 HᵀR⁻¹r，观测权重已包含在其中。
  observation.HTH_.noalias() =
      jacobian.transpose() * wheel_information_ * jacobian;
  observation.HTr_.noalias() =
      jacobian.transpose() * wheel_information_ * residual;
}

// 处理一帧同步扫描：逐段 IMU 预测、轮速/NHC 更新，最后完成点云去畸变。
// 一帧可进行多次轮速更新，每个传播区间只使用其中最新的一条新轮速。
void ImuProcess::ProcessWithWheelOdometry(
    MeasureGroup& meas, ESKF& kf_state, CloudPtr& scan,
    bool wheel_updates_allowed) {
  scan.reset();
  // 初始化沿用原有 Process，尚需初始化时不进入轮速融合流程。
  if (imu_need_init_) {
    Process(meas, kf_state, scan);
    return;
  }
  const double scan_end_time = meas.lidar_end_time_;
  // 上帧保留的实际 IMU 测量作为左端输入，其时间可早于状态已传播到的 cursor_time。
  auto previous_imu = *last_imu_;
  double cursor_time = last_propagated_time_;
  if (cursor_time <= 0 || scan_end_time <= cursor_time) return;
  DeskewTrajectory trajectory;
  Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
  Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
  Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
  Vec3d mean_gyro = previous_imu.angular_velocity;
  Vec3d mean_acc = previous_imu.linear_acceleration;
  // 推进到当前区间末尾；先预测，再按更新许可进行至多一次轮速/NHC 更新。
  auto propagate = [&](double end_time, const Vec3d& gyro, const Vec3d& acc) {
    if (end_time <= cursor_time) return true;
    const SE3 begin_pose = kf_state.GetX().GetPose();
    kf_state.Predict(end_time - cursor_time, Q_, gyro, acc);
    kf_state.SetTime(end_time);
    const SE3 predicted_pose = kf_state.GetX().GetPose();

    // 消费不晚于区间末尾的轮速，只保留当前区间内最新的一条。
    // pop_front 同时推进读取位置，已经取出的测量不会在后续区间重复融合。
    OdomPtr latest_odom;
    while (!meas.odom_.empty() &&
           meas.odom_.front()->timestamp <= end_time) {
      auto measurement = std::move(meas.odom_.front());
      meas.odom_.pop_front();
      if (measurement->timestamp >= cursor_time) {
        latest_odom = std::move(measurement);
      }
    }
    if (latest_odom && wheel_updates_allowed) {
      // 固定本次更新的速度和陀螺输入，观测回调每次迭代使用同一份测量。
      wheel_forward_speed_ = latest_odom->linear_velocity.x();
      wheel_gyro_ = gyro;
      kf_state.Update(ESKF::ObsType::WHEEL_SPEED, 1.0);
    }

    // 分开记录末尾预测与后验，轮速引起的位姿修正从区间末尾开始生效。
    if (!trajectory.Append(cursor_time, begin_pose, end_time, predicted_pose,
                           kf_state.GetX().GetPose())) {
      return false;
    }
    cursor_time = end_time;
    last_propagated_time_ = cursor_time;
    return true;
  };

  // IMU 已完成预处理，直接用相邻测量均值传播；顺序遍历即可，无需逐条 pop。
  // 本帧测量包由下次同步清空，last_imu_ 单独保留跨帧所需的左端测量。
  for (const auto& imu : meas.imu_) {
    const double time = imu->timestamp;
    if (time <= previous_imu.timestamp) continue;
    if (time > scan_end_time) break;
    const auto& current_imu = *imu;
    mean_gyro =
        0.5 * (previous_imu.angular_velocity + current_imu.angular_velocity);
    mean_acc =
        0.5 * (previous_imu.linear_acceleration +
               current_imu.linear_acceleration);
    if (!propagate(time, mean_gyro, mean_acc)) return;
    previous_imu = current_imu;
    last_imu_ = imu;
  }
  // 沿用最近的传播输入补齐到扫描末尾，已到达末尾时不重复预测。
  if (!propagate(scan_end_time, mean_gyro, mean_acc)) return;
  last_lidar_end_time_ = scan_end_time;

  scan = meas.scan_;
  if (!scan) return;
  // 使用融合后的扫描末尾 IMU 位姿作为去畸变参考。
  DeskewPointCloud(meas.lidar_begin_time_, trajectory,
                   kf_state.GetX().GetPose(), *scan);
}

// 原地补偿到扫描末尾的 LiDAR 坐标系；轨迹及参考位姿均采用真实 IMU 的 T_WI。
void ImuProcess::DeskewPointCloud(double scan_begin_time,
                                  const DeskewTrajectory& trajectory,
                                  const SE3& world_from_imu_end,
                                  PointCloudType& scan) const {
  const SE3 imu_from_lidar(SO3(R_lidar_imu_), t_lidar_mu_);
  // 组合世界系到扫描末尾 LiDAR 系的变换：T_Lend_W = T_IL⁻¹ * T_WIend⁻¹。
  const SE3 lidar_end_from_world =
      imu_from_lidar.inverse() * world_from_imu_end.inverse();
  std::size_t written = 0;
  const std::size_t original_size = scan.size();
  for (auto point : scan.points) {
    if (!std::isfinite(point.time)) continue;
    // point.time 是距扫描起点的秒偏移，转为与 IMU 一致的时间戳后查询位姿。
    const double point_time = scan_begin_time + point.time;
    const auto pose = trajectory.EvaluatePose(point_time);
    if (!pose) continue;
    // 点坐标依次经过 L(t) → I(t) → 世界系 → I(end) → L(end)。
    const Vec3d compensated =
        lidar_end_from_world * *pose * imu_from_lidar *
        Vec3d(point.x, point.y, point.z);
    point.x = static_cast<float>(compensated.x());
    point.y = static_cast<float>(compensated.y());
    point.z = static_cast<float>(compensated.z());
    // 原地压紧有效点，保持点的原有顺序，未被轨迹覆盖的点不进入输出。
    scan.points[written++] = point;
  }
  scan.resize(written);
  if (written != original_size) {
    LOG_EVERY_N(WARNING, 100) << "Dropped " << original_size - written
                              << " points outside IMU trajectory";
  }
}

}  // namespace lightning
