#ifndef LIGHTNING_KEYFRAME_MAP_H
#define LIGHTNING_KEYFRAME_MAP_H

#include <cstdint>
#include <memory>
#include <vector>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

/// Copy of the fields required to export a map without holding backend locks.
struct KeyframeMapEntry {
    std::uint64_t id = 0;
    double timestamp = 0.0;
    std::shared_ptr<const PointCloudType> cloud;
    SE3 lio_pose = SE3();
    SE3 optimized_pose = SE3();
};

/// Builds a point cloud map from a stable PoseGraph keyframe snapshot.
CloudPtr BuildKeyframeMap(const std::vector<KeyframeMapEntry>& keyframes,
                          bool use_lio_pose,
                          bool use_voxel = true,
                          float resolution = 0.1F);

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_MAP_H
