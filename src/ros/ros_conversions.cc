#include "ros/ros_conversions.h"

#include <cmath>
#include <cstdint>

namespace lightning::ros {

builtin_interfaces::msg::Time ToRosTime(double seconds) {
    builtin_interfaces::msg::Time stamp;
    if (!std::isfinite(seconds)) {
        return stamp;
    }

    const std::int64_t total_nanoseconds = static_cast<std::int64_t>(std::llround(seconds * 1e9));
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    std::int64_t sec = total_nanoseconds / kNanosecondsPerSecond;
    std::int64_t nanosec = total_nanoseconds % kNanosecondsPerSecond;
    if (nanosec < 0) {
        --sec;
        nanosec += kNanosecondsPerSecond;
    }
    stamp.sec = static_cast<std::int32_t>(sec);
    stamp.nanosec = static_cast<std::uint32_t>(nanosec);
    return stamp;
}

geometry_msgs::msg::TransformStamped ToTransform(const loc::LocalizationResult& result,
                                                 const std::string& frame_id,
                                                 const std::string& child_frame_id) {
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id = frame_id;
    message.header.stamp = ToRosTime(result.timestamp_);
    message.child_frame_id = child_frame_id;

    const auto& translation = result.pose_.translation();
    const auto& quaternion = result.pose_.unit_quaternion();
    message.transform.translation.x = translation.x();
    message.transform.translation.y = translation.y();
    message.transform.translation.z = translation.z();
    message.transform.rotation.x = quaternion.x();
    message.transform.rotation.y = quaternion.y();
    message.transform.rotation.z = quaternion.z();
    message.transform.rotation.w = quaternion.w();
    return message;
}

nav_msgs::msg::OccupancyGrid ToOccupancyGrid(const GridMapData& map, const std::string& frame_id) {
    nav_msgs::msg::OccupancyGrid message;
    message.header.frame_id = frame_id;
    message.info.resolution = map.resolution;
    message.info.width = map.width;
    message.info.height = map.height;
    message.info.origin.position.x = map.origin.x();
    message.info.origin.position.y = map.origin.y();
    message.info.origin.orientation.w = 1.0;
    message.data = map.cells;
    return message;
}

}  // namespace lightning::ros
