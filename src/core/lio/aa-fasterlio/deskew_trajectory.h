#ifndef LIGHTNING_CORE_LIO_AA_FASTERLIO_DESKEW_TRAJECTORY_H_
#define LIGHTNING_CORE_LIO_AA_FASTERLIO_DESKEW_TRAJECTORY_H_

#include <optional>
#include <vector>

#include "common/eigen_types.h"

namespace lightning {

// 保存一帧扫描期间的 IMU 运动轨迹，供点云去畸变查询。
// 时间与 IMU 时间戳使用同一基准，单位为秒；位姿为 T_WI（IMU 系到世界系）。
// 区间内部使用预测运动，轮速观测带来的修正从区间末尾开始生效。
class DeskewTrajectory {
 public:
  // 追加一个传播区间，后续区间的起点时间须等于上一段的终点时间。
  // 三个位姿依次为起点后验、终点预测、终点后验；无轮速修正时后两者相同。
  // 时间、位姿或连续性检查未通过时返回 false，不追加区间。
  bool Append(double begin_time, const SE3& begin_posterior,
              double end_time, const SE3& end_prediction,
              const SE3& end_posterior);
  // 查询 time 时刻的 T_WI：区间内部插值，端点取对应的后验位姿。
  // 空轨迹、无效时间或超出覆盖范围（允许边界舍入容差）时返回 std::nullopt。
  std::optional<SE3> EvaluatePose(double time) const;

 private:
  // 一段 IMU 预测及其末尾观测修正，分别保留修正前后的位姿。
  struct Interval {
    double begin_time;     // 区间开始时间，单位秒。
    double end_time;       // 区间结束时间，单位秒。
    SE3 begin_posterior;   // 起点位姿，包含此前的观测修正。
    SE3 end_prediction;    // IMU 预测到终点、尚未进行本次轮速更新的位姿。
    SE3 end_posterior;     // 终点完成轮速更新后的位姿，无更新时等于预测位姿。
    Vec3d rotation_delta;  // 起点到预测终点的 SO3 对数，旋转向量，单位弧度。
  };
  // 按时间连续排列的运动区间，支持按区间终点进行二分查找。
  std::vector<Interval> intervals_;
};

}  // namespace lightning

#endif  // LIGHTNING_CORE_LIO_AA_FASTERLIO_DESKEW_TRAJECTORY_H_
