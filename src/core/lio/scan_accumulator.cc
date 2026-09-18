#include "core/lio/scan_accumulator.h"

#include <algorithm>

namespace lightning {

void ScanAccumulator::AddSelected(const LIOResult& result) {
    if (!result.state.pose_is_valid || !result.state.pose.matrix().allFinite() ||
        !result.body_from_lidar.matrix().allFinite() || !result.cloud || result.cloud->empty() ||
        options_.max_scans == 0) {
        return;
    }

    for (const PointType& point : result.cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.intensity)) {
            return;
        }
    }

    Entry entry;
    entry.state = result.state;
    entry.body_from_lidar = result.body_from_lidar;
    entry.cloud = std::make_shared<const PointCloudType>(*result.cloud);

    if (selected_scans_.size() < options_.max_scans) {
        selected_scans_.emplace_back(std::move(entry));
        return;
    }

    const Entry& newest = selected_scans_.back();
    const SE3 newest_lidar_pose = newest.state.pose * newest.body_from_lidar;
    const SE3 current_lidar_pose = result.state.pose * result.body_from_lidar;
    const SE3 delta = newest_lidar_pose.inverse() * current_lidar_pose;
    if (delta.translation().norm() < options_.replacement_translation_threshold ||
        delta.so3().log().norm() < options_.replacement_rotation_threshold) {
        return;
    }

    selected_scans_.pop_front();
    selected_scans_.emplace_back(std::move(entry));
}

// NOTE: 当前实现与原版不同
CloudPtr ScanAccumulator::BuildProjectedCloud(const LIOResult& current) const {
    if (!current.cloud || current.cloud->empty() || !current.state.pose_is_valid ||
        !current.state.pose.matrix().allFinite() || !current.body_from_lidar.matrix().allFinite()) {
        return nullptr;
    }

    for (const PointType& point : current.cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.intensity)) {
            return nullptr;
        }
    }

    auto projected = std::make_shared<PointCloudType>(*current.cloud);
    // 原版是直接取卡尔曼滤波器的最新位姿，而这里是取得当前输入的位姿
    const SE3 current_lidar_pose = current.state.pose * current.body_from_lidar;
    for (const auto& entry : selected_scans_) {
        if (!entry.cloud) {
            continue;
        }

        const SE3 history_to_current =
            current_lidar_pose.inverse() * (entry.state.pose * entry.body_from_lidar);
        const std::size_t point_limit = std::min(options_.max_points_per_scan, entry.cloud->size());
        for (std::size_t index = 0; index < point_limit; ++index) {
            const Vec3d point = history_to_current * ToVec3d(entry.cloud->points[index]);
            PointType projected_point = entry.cloud->points[index];
            projected_point.x = static_cast<float>(point.x());
            projected_point.y = static_cast<float>(point.y());
            projected_point.z = static_cast<float>(point.z());
            projected->push_back(projected_point);
        }
    }

    projected->height = 1;
    projected->width = static_cast<std::uint32_t>(projected->size());
    projected->is_dense = false;
    return projected;
}

void ScanAccumulator::TrimToLimit() {
    while (selected_scans_.size() > options_.max_scans) {
        selected_scans_.pop_front();
    }
}

}  // namespace lightning
