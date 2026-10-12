#ifndef LIGHTNING_WHEEL_ODOMETRY_DATA_H
#define LIGHTNING_WHEEL_ODOMETRY_DATA_H

#include <cstdint>
#include <string>

#include "common/eigen_types.h"

namespace lightning {

/// Transport-independent representation of a wheel odometry message.
struct WheelOdometryData {
    std::int64_t timestamp_ns = 0;
    std::string frame_id;
    std::string child_frame_id;

    SE3 pose = SE3();
    Vec3d linear_velocity = Vec3d::Zero();
    Vec3d angular_velocity = Vec3d::Zero();

    Mat6d pose_covariance = Mat6d::Zero();
    Mat6d twist_covariance = Mat6d::Zero();
    bool has_pose = false;
    bool has_twist = false;
    bool has_pose_covariance = false;
    bool has_twist_covariance = false;
};

}  // namespace lightning

#endif  // LIGHTNING_WHEEL_ODOMETRY_DATA_H
