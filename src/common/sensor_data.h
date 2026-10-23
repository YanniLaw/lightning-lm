#pragma once

#include "common/point_def.h"

namespace lightning {

enum class LidarType {
    AVIA = 1,
    VELO32 = 2,
    OUST64 = 3,
    ROBOSENSE = 4,
};

// Preprocessed cloud and scan start time. After submission only the LIO worker
// may modify cloud; copying this packet never copies the point array.
struct TimedPointCloud {
  double timestamp = 0.0;  // Seconds.
  CloudPtr cloud;
};

using TimedPointCloudPtr = std::shared_ptr<TimedPointCloud>;

enum class InputResult {
    Accepted,
    NotRunning,
    InvalidData,
    QueueFull,
    TimeDiscontinuity,
};

}  // namespace lightning
