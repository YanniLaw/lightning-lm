#pragma once

#include <cstdint>
#include <vector>

namespace lightning {

enum class LidarType {
    AVIA = 1,
    VELO32 = 2,
    OUST64 = 3,
    ROBOSENSE = 4,
};

struct RawLidarPoint {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    std::int64_t time_offset_ns = 0;
    std::uint16_t ring = 0;
    std::uint8_t tag = 0;
    bool has_time = false;
};

struct TimedPointCloudData {
    std::int64_t timestamp_ns = 0;
    LidarType lidar_type = LidarType::AVIA;
    bool has_point_time = false;
    std::vector<RawLidarPoint> points;
};

enum class InputResult {
    Accepted,
    NotRunning,
    InvalidData,
    QueueFull,
    TimeDiscontinuity,
};

}  // namespace lightning
