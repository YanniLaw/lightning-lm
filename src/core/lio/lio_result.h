#ifndef LIGHTNING_LIO_RESULT_H
#define LIGHTNING_LIO_RESULT_H

#include <memory>
#include <optional>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

enum class LIOUpdateType {
    kPrediction,
    kScanMatched,
};

/// State produced by a LIO frontend in the local/body coordinate convention.
struct LIOState {
    double timestamp = 0.0;
    SE3 pose = SE3();  // T_local_body.

    std::optional<Vec3d> velocity;
    std::optional<Vec3d> gyro_bias;
    std::optional<Vec3d> accel_bias;
    std::optional<Vec3d> gravity;

    bool pose_is_valid = false;
    bool lidar_odom_reliable = true;
};

/// Immutable output from one frontend update.
struct LIOResult {
    LIOState state;
    LIOUpdateType update_type = LIOUpdateType::kPrediction;

    // All clouds below are in the LiDAR coordinate frame at state.timestamp.
    std::shared_ptr<const PointCloudType> cloud;
    std::shared_ptr<const PointCloudType> display_cloud;
    std::shared_ptr<const PointCloudType> projected_cloud;

    // Transform from the LiDAR frame to the body frame used by state.pose.
    SE3 body_from_lidar = SE3();

    // True only when this scan-matched result should become a graph node.
    bool keyframe_selected = false;
};

}  // namespace lightning

#endif  // LIGHTNING_LIO_RESULT_H
