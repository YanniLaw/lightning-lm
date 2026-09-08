#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "common/eigen_types.h"

namespace lightning {

/// A transport-independent occupancy-grid snapshot.
/// Cells use the ROS occupancy convention: -1 unknown, 0 free, 100 occupied.
struct GridMapData {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    float resolution = 0.0F;
    Vec2d origin = Vec2d::Zero();
    std::vector<std::int8_t> cells;
};

using GridMapDataPtr = std::shared_ptr<const GridMapData>;

}  // namespace lightning
