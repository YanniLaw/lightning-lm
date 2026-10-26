#pragma once

#include "common/point_def.h"

namespace lightning {

enum class LidarType {
    AVIA = 1,
    VELO32 = 2,
    OUST64 = 3,
    ROBOSENSE = 4,
};

enum class InputResult {
    Accepted,
    NotRunning,
    InvalidData,
    QueueFull,
    TimeDiscontinuity,
};

}  // namespace lightning
