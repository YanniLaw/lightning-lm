#include "core/lio/aa-fasterlio/deskew_trajectory.h"

#include <algorithm>
#include <cmath>

namespace lightning {

bool DeskewTrajectory::Append(double begin_time,
                             const SE3& begin_posterior,
                             double end_time, const SE3& end_prediction,
                             const SE3& end_posterior) {
  if (!std::isfinite(begin_time) || !std::isfinite(end_time) ||
      begin_time < 0 || end_time <= begin_time ||
      (!intervals_.empty() && begin_time != intervals_.back().end_time) ||
      !begin_posterior.matrix().allFinite() ||
      !end_prediction.matrix().allFinite() ||
      !end_posterior.matrix().allFinite())
    return false;
  // 同一区间内所有点共用这个旋转增量，避免逐点重复计算相对旋转及其对数。
  const Vec3d rotation_delta =
      (begin_posterior.so3().inverse() * end_prediction.so3()).log();
  intervals_.push_back({begin_time, end_time, begin_posterior, end_prediction,
                        end_posterior, rotation_delta});
  return true;
}

std::optional<SE3> DeskewTrajectory::EvaluatePose(double time) const {
  // 仅用于处理 double 秒在边界比较时的舍入误差，不代表传感器时间精度。
  constexpr double kBoundaryTolerance = 1e-6;
  if (intervals_.empty() || !std::isfinite(time) || time < 0)
    return std::nullopt;
  const auto& first = intervals_.front();
  const auto& last = intervals_.back();
  // 覆盖范围之外不做轨迹外推，容差范围内的查询按相邻端点处理。
  if ((time < first.begin_time &&
       first.begin_time - time > kBoundaryTolerance) ||
      (time > last.end_time && time - last.end_time > kBoundaryTolerance))
    return std::nullopt;
  if (time <= first.begin_time ||
      time - first.begin_time <= kBoundaryTolerance)
    return first.begin_posterior;
  if (time >= last.end_time || last.end_time - time <= kBoundaryTolerance)
    return last.end_posterior;
  // 找到第一个 end_time >= time 的区间，即当前查询所属的运动区间。
  auto interval =
      std::lower_bound(intervals_.begin(), intervals_.end(), time,
                       [](const Interval& item, double time) {
                         return item.end_time < time;
                       });
  // 轮速更新的修正从区间末尾开始生效，端点直接使用后验位姿。
  if (interval->end_time - time <= kBoundaryTolerance)
    return interval->end_posterior;
  if (time - interval->begin_time <= kBoundaryTolerance)
    return interval->begin_posterior;
  // 区间内部采用位置线性插值和 SO3 姿态插值，不将末尾的观测修正摊入区间。
  const double alpha =
      (time - interval->begin_time) /
      (interval->end_time - interval->begin_time);
  return SE3(interval->begin_posterior.so3() *
                 SO3::exp(alpha * interval->rotation_delta),
             (1.0 - alpha) * interval->begin_posterior.translation() +
                 alpha * interval->end_prediction.translation());
}

}  // namespace lightning
