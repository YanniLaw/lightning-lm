#ifndef LIGHTNING_COMMON_WHEEL_ODOMETRY_OPTIONS_H_
#define LIGHTNING_COMMON_WHEEL_ODOMETRY_OPTIONS_H_

#include "common/eigen_types.h"

namespace lightning {

// Fixed installation and observation settings. All durations use seconds.
struct WheelOdometryOptions {
  // Enables wheel input and fusion via system.with_wheel_odometry_fusion.
  bool fusion_enabled = false;
  // Internal B -> L, obtained by inverting the YAML L -> B pose once.
  SE3 lidar_from_wheel;  // Required when enabled.
  double wheel_time_offset = 0.0;
  double default_velocity_variance = 0.01;  // (m/s)^2.
  double nhc_lateral_variance = 0.01;       // (m/s)^2.
  double nhc_vertical_variance = 0.04;      // (m/s)^2.
};

}  // namespace lightning

#endif  // LIGHTNING_COMMON_WHEEL_ODOMETRY_OPTIONS_H_
