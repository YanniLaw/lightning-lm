#pragma once

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

/// Data produced by the LIO frontend for external consumers.
///
/// This intentionally contains only project-internal types. ROS adapters,
/// backend optimization and loop-closing consumers can register independent
/// callbacks without coupling LaserMapping to a particular transport.
struct LIOData {
    SE3 pose;
    double timestamp = 0.0;
    std::shared_ptr<const PointCloudType> registered_cloud;
    std::shared_ptr<const PointCloudType> ivox_map;
};

}  // namespace lightning
