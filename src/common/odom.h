#ifndef LIGHTNING_COMMON_ODOM_H_
#define LIGHTNING_COMMON_ODOM_H_

#include <memory>

#include "common/eigen_types.h"

namespace lightning {

// Wheel-encoder odometry expressed in the actual IMU axes. Covariance is ignored.
// The timestamp is corrected before submission; consumers only read the sample.
struct Odom {
  double timestamp = 0.0;  // Seconds.
  SE3 pose = SE3();  // IMU pose in the odometry world; not fused.
  // Velocity at the wheel center, expressed in IMU axes.
  Vec3d linear_velocity = Vec3d::Zero();
  Vec3d angular_velocity = Vec3d::Zero();  // Encoder angular velocity; not fused.
};

using OdomPtr = std::shared_ptr<Odom>;

}  // namespace lightning

#endif  // LIGHTNING_COMMON_ODOM_H_
