#pragma once

#include <memory>

#include "common/eigen_types.h"
#include "common/point_def.h"
#include "core/lio/lio_result.h"

namespace lightning {

/// Data produced by the LIO frontend for external consumers.
///
/// This intentionally contains only project-internal types. ROS adapters,
/// backend optimization and loop-closing consumers can register independent
/// callbacks without coupling a frontend implementation to a transport.
struct LIOData {
    LIOState state;
    std::shared_ptr<const PointCloudType> registered_cloud;  // Frontend local world.
    std::shared_ptr<const PointCloudType> ivox_map;  // Frontend local world.
};

}  // namespace lightning
