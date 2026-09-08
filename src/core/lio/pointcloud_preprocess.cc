#include "core/lio/pointcloud_preprocess.h"

#include <cmath>
#include <vector>

#include <glog/logging.h>

namespace lightning {

void PointCloudPreprocess::Set(LidarType lidar_type, double blind, int point_filter_num) {
    lidar_type_ = lidar_type;
    blind_ = blind;
    point_filter_num_ = point_filter_num;
}

bool PointCloudPreprocess::Process(const TimedPointCloudData& scan, PointCloudType::Ptr& pcl_out) const {
    if (scan.points.empty() || point_filter_num_ <= 0 || scan.lidar_type != lidar_type_) {
        return false;
    }

    if (!pcl_out) {
        // PCL 1.10 uses boost::shared_ptr while newer releases may use
        // std::shared_ptr.  reset(new ...) is compatible with both APIs.
        pcl_out.reset(new PointCloudType);
    }
    pcl_out->clear();
    pcl_out->header.stamp = static_cast<std::uint64_t>(scan.timestamp_ns);
    pcl_out->height = 1;
    pcl_out->is_dense = false;

    PointCloudType& output = *pcl_out;
    const bool processed = scan.lidar_type == LidarType::AVIA ? ProcessLivox(scan, output)
                                                               : ProcessStandard(scan, output);
    if (!processed) {
        pcl_out->clear();
        pcl_out->header.stamp = static_cast<std::uint64_t>(scan.timestamp_ns);
        pcl_out->height = 1;
        pcl_out->is_dense = false;
        return false;
    }

    pcl_out->width = static_cast<std::uint32_t>(pcl_out->size());
    return !pcl_out->empty();
}

bool PointCloudPreprocess::ProcessLivox(const TimedPointCloudData& scan, PointCloudType& pcl_out) const {
    if (scan.points.size() < 2) {
        return false;
    }

    pcl_out.reserve(scan.points.size());
    const double blind_squared = blind_ * blind_;

    for (std::size_t i = 1; i < scan.points.size(); ++i) {
        if (i % static_cast<std::size_t>(point_filter_num_) != 0) {
            continue;
        }

        const RawLidarPoint& current = scan.points[i];
        const RawLidarPoint& previous = scan.points[i - 1];
        if (!IsInHeightRoi(current)) {
            continue;
        }

        const double range_squared = static_cast<double>(current.x) * current.x +
                                     static_cast<double>(current.y) * current.y +
                                     static_cast<double>(current.z) * current.z;
        // Keep the historical Livox acceptance rule while removing the dependency
        // on an uninitialized or concurrently written converted point.
        if (std::abs(current.x - previous.x) > 1e-7F || std::abs(current.y - previous.y) > 1e-7F ||
            (std::abs(current.z - previous.z) > 1e-7F && range_squared > blind_squared)) {
            pcl_out.push_back(ToPoint(current, static_cast<double>(current.time_offset_ns) / 1e6));
        }
    }

    return true;
}

bool PointCloudPreprocess::ProcessStandard(const TimedPointCloudData& scan, PointCloudType& pcl_out) const {
    if (scan.lidar_type != LidarType::VELO32 && scan.lidar_type != LidarType::OUST64 &&
        scan.lidar_type != LidarType::ROBOSENSE) {
        LOG(ERROR) << "Unsupported standard lidar type";
        return false;
    }

    pcl_out.reserve(scan.points.size());
    const double blind_squared = blind_ * blind_;

    std::vector<bool> first_point(num_scans_, true);
    std::vector<double> first_yaw(num_scans_, 0.0);
    std::vector<double> last_time(num_scans_, 0.0);

    for (std::size_t i = 0; i < scan.points.size(); ++i) {
        const RawLidarPoint& raw_point = scan.points[i];
        double time_ms = raw_point.has_time ? static_cast<double>(raw_point.time_offset_ns) / 1e6 : 0.0;

        if (scan.lidar_type == LidarType::VELO32 && !scan.has_point_time) {
            if (raw_point.ring >= static_cast<std::uint16_t>(num_scans_)) {
                continue;
            }

            const std::size_t ring = raw_point.ring;
            const double yaw = std::atan2(raw_point.y, raw_point.x) * 57.2957;
            if (first_point[ring]) {
                first_yaw[ring] = yaw;
                first_point[ring] = false;
                last_time[ring] = 0.0;
                continue;
            }

            time_ms = yaw <= first_yaw[ring] ? (first_yaw[ring] - yaw) / 3.61
                                             : (first_yaw[ring] - yaw + 360.0) / 3.61;
            if (time_ms < last_time[ring]) {
                time_ms += 360.0 / 3.61;
            }
            last_time[ring] = time_ms;
        }

        if (i % static_cast<std::size_t>(point_filter_num_) != 0) {
            continue;
        }

        const double range_squared = static_cast<double>(raw_point.x) * raw_point.x +
                                     static_cast<double>(raw_point.y) * raw_point.y +
                                     static_cast<double>(raw_point.z) * raw_point.z;
        if (range_squared < blind_squared) {
            continue;
        }
        if (scan.lidar_type != LidarType::VELO32 && !IsInHeightRoi(raw_point)) {
            continue;
        }

        pcl_out.push_back(ToPoint(raw_point, time_ms));
    }

    return true;
}

bool PointCloudPreprocess::IsInHeightRoi(const RawLidarPoint& point) const {
    return point.z >= height_min_ && point.z <= height_max_;
}

PointType PointCloudPreprocess::ToPoint(const RawLidarPoint& point, double time_ms) {
    PointType converted;
    converted.x = point.x;
    converted.y = point.y;
    converted.z = point.z;
    converted.intensity = point.intensity;
    converted.time = time_ms;
    return converted;
}

}  // namespace lightning
