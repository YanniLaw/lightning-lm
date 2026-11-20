// Copyright 2026

#include "core/descriptor/scan_context_descriptor.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace lightning {

ScanContextDescriptor::ScanContextDescriptor(Options options)
    : descriptor_manager_(std::move(options)) {}

std::vector<LoopDescriptor::Match> ScanContextDescriptor::DetectAndAdd(
    const CloudConstPtr& cloud, const SE3& local_from_query,
    std::uint64_t keyframe_id, std::uint64_t reference_id_limit) {
  const Mat3d rotation = local_from_query.rotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const SO3 level_from_imu =
      SO3::exp(Vec3d(0.0, 0.0, -yaw)) * local_from_query.so3();

  pcl::PointCloud<scan_context::SCPointType> input;
  input.reserve(cloud->size());
  for (const auto& point : *cloud) {
    scan_context::SCPointType converted;
    converted.getVector3fMap() =
        (level_from_imu * point.getVector3fMap().cast<double>()).cast<float>();
    converted.intensity = point.intensity;
    input.push_back(converted);
  }

  const auto eligible_end = std::lower_bound(
      keyframe_ids_.begin(), keyframe_ids_.end(), reference_id_limit);
  const std::size_t eligible_count = eligible_end - keyframe_ids_.begin();
  const std::size_t previous_count = descriptor_manager_.GetNumDescriptors();
  const auto match = descriptor_manager_.DetectAndAdd(input, eligible_count);
  if (descriptor_manager_.GetNumDescriptors() > previous_count) {
    keyframe_ids_.push_back(keyframe_id);
    level_from_imu_rotations_.push_back(level_from_imu);
  }
  if (!match) {
    return {};
  }

  const SO3 reference_from_query_rotation =
      level_from_imu_rotations_[match->index].inverse() *
      SO3::exp(Vec3d(0.0, 0.0, -match->yaw_shift_rad)) * level_from_imu;
  const auto reference_id = keyframe_ids_[match->index];
  LOG(INFO) << "Scan Context loop candidate: current=" << keyframe_id
            << ", reference=" << reference_id
            << ", distance=" << match->distance
            << ", yaw=" << std::remainder(-match->yaw_shift_rad, 2.0 * M_PI) *
                               180.0 / M_PI
            << " deg";
  return {{reference_id, std::nullopt, reference_from_query_rotation}};
}

}  // namespace lightning
