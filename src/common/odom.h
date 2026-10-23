#ifndef LIGHTNING_COMMON_ODOM_H_
#define LIGHTNING_COMMON_ODOM_H_

#include <memory>

#include "common/eigen_types.h"

namespace lightning {

// Wheel-encoder odometry. Message covariance is ignored.
struct Odom {
  double timestamp = 0.0;  // Seconds.
  SE3 pose = SE3();
  Vec3d linear_velocity = Vec3d::Zero();
  Vec3d angular_velocity = Vec3d::Zero();
};

using OdomPtr = std::shared_ptr<Odom>;

}  // namespace lightning

#endif  // LIGHTNING_COMMON_ODOM_H_
