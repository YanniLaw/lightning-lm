#include "ros/sensor_bridge.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>

#include "common/point_def.h"

namespace lightning::ros {
namespace {

std::int64_t ToNanoseconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
}

bool IsFinite(const RawLidarPoint& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           std::isfinite(point.intensity);
}

template <typename PointT, typename Converter>
bool CopyPointCloud(const sensor_msgs::msg::PointCloud2& message, LidarType lidar_type,
                    TimedPointCloudData& output, Converter&& converter) {
    pcl::PointCloud<PointT> source;
    try {
        pcl::fromROSMsg(message, source);
    } catch (...) {
        return false;
    }

    if (source.empty()) {
        return false;
    }

    output.timestamp_ns = ToNanoseconds(message.header.stamp);
    output.lidar_type = lidar_type;
    output.has_point_time = false;
    output.points.clear();
    output.points.reserve(source.size());

    for (const auto& point : source) {
        RawLidarPoint converted;
        if (!converter(point, converted) || !IsFinite(converted)) {
            return false;
        }
        output.has_point_time = output.has_point_time || converted.has_time;
        output.points.emplace_back(converted);
    }

    return !output.points.empty();
}

}  // namespace

bool ToImuData(const sensor_msgs::msg::Imu& message, IMUPtr& output) {
    const double timestamp = static_cast<double>(ToNanoseconds(message.header.stamp)) * 1e-9;
    if (!std::isfinite(timestamp) || !std::isfinite(message.angular_velocity.x) ||
        !std::isfinite(message.angular_velocity.y) || !std::isfinite(message.angular_velocity.z) ||
        !std::isfinite(message.linear_acceleration.x) || !std::isfinite(message.linear_acceleration.y) ||
        !std::isfinite(message.linear_acceleration.z)) {
        return false;
    }

    IMUPtr converted = std::make_shared<IMU>();
    converted->timestamp = timestamp;
    converted->angular_velocity =
        Vec3d(message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
    converted->linear_acceleration =
        Vec3d(message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z);
    output = std::move(converted);
    return true;
}

bool ToTimedPointCloudData(const livox_ros_driver2::msg::CustomMsg& message,
                           TimedPointCloudData& output) {
    if (message.point_num == 0 || message.point_num > message.points.size()) {
        return false;
    }

    output.timestamp_ns = ToNanoseconds(message.header.stamp);
    output.lidar_type = LidarType::AVIA;
    output.has_point_time = true;
    output.points.clear();
    output.points.reserve(message.point_num);

    for (std::size_t i = 0; i < message.point_num; ++i) {
        const auto& point = message.points[i];
        RawLidarPoint converted;
        converted.x = point.x;
        converted.y = point.y;
        converted.z = point.z;
        converted.intensity = static_cast<float>(point.reflectivity);
        converted.time_offset_ns = static_cast<std::int64_t>(point.offset_time);
        converted.ring = point.line;
        converted.tag = point.tag;
        converted.has_time = true;
        if (!IsFinite(converted)) {
            return false;
        }
        output.points.emplace_back(converted);
    }

    return !output.points.empty();
}

bool ToTimedPointCloudData(const sensor_msgs::msg::PointCloud2& message, LidarType lidar_type,
                           double velodyne_time_scale, TimedPointCloudData& output) {
    if (!std::isfinite(velodyne_time_scale) || velodyne_time_scale < 0.0) {
        return false;
    }

    switch (lidar_type) {
        case LidarType::OUST64: {
            const bool converted = CopyPointCloud<ouster_ros::Point>(
                message, lidar_type, output, [](const ouster_ros::Point& point, RawLidarPoint& converted) {
                    converted.x = point.x;
                    converted.y = point.y;
                    converted.z = point.z;
                    converted.intensity = point.intensity;
                    converted.time_offset_ns = static_cast<std::int64_t>(point.t);
                    converted.ring = point.ring;
                    converted.has_time = true;
                    return true;
                });
            return converted;
        }
        case LidarType::ROBOSENSE: {
            const double header_time = static_cast<double>(ToNanoseconds(message.header.stamp)) * 1e-9;
            if (!std::isfinite(header_time)) {
                return false;
            }
            const bool converted = CopyPointCloud<PointRobotSense>(
                message, lidar_type, output,
                [header_time](const PointRobotSense& point, RawLidarPoint& converted) {
                    if (!std::isfinite(point.timestamp) || point.timestamp <= 0.0) {
                        return false;
                    }
                    converted.x = point.x;
                    converted.y = point.y;
                    converted.z = point.z;
                    converted.intensity = point.intensity;
                    const double offset_ns = (point.timestamp - header_time) * 1e9;
                    if (!std::isfinite(offset_ns) ||
                        offset_ns < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
                        offset_ns > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
                        return false;
                    }
                    converted.time_offset_ns = static_cast<std::int64_t>(std::llround(offset_ns));
                    converted.has_time = true;
                    return true;
                });
            return converted;
        }
        case LidarType::VELO32: {
            const bool converted = CopyPointCloud<velodyne_ros::Point>(
                message, lidar_type, output,
                [velodyne_time_scale](const velodyne_ros::Point& point, RawLidarPoint& converted) {
                    converted.x = point.x;
                    converted.y = point.y;
                    converted.z = point.z;
                    converted.intensity = point.intensity;
                    converted.ring = point.ring;
                    if (!std::isfinite(point.time)) {
                        return false;
                    }
                    const double time_ms = static_cast<double>(point.time) * velodyne_time_scale;
                    if (std::isfinite(time_ms) && time_ms >= 0.0 &&
                        time_ms <= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e6) {
                        converted.time_offset_ns = static_cast<std::int64_t>(std::llround(time_ms * 1e6));
                        converted.has_time = point.time > 0.0F;
                    }
                    return true;
                });
            return converted;
        }
        case LidarType::AVIA:
            return false;
    }

    return false;
}

}  // namespace lightning::ros
