// Copyright 2026

#include "core/descriptor/std_descriptor.h"

#include <glog/logging.h>

#include <algorithm>
#include <memory>
#include <utility>

#include "core/descriptor/std/STDesc.h"

namespace lightning {
namespace {

std_descriptor::STDescManager MakeDescriptorManager(
    const StdDescriptor::Options& options) {
  std_descriptor::ConfigSetting config{};
  config.maximum_corner_num_ = options.maximum_corner_num;
  config.plane_merge_normal_thre_ = options.plane_merge_normal_thre;
  config.plane_detection_thre_ = options.plane_detection_thre;
  config.voxel_size_ = options.voxel_size;
  config.voxel_init_num_ = options.voxel_init_num;
  config.proj_image_resolution_ = options.proj_image_resolution;
  config.proj_dis_min_ = options.proj_dis_min;
  config.proj_dis_max_ = options.proj_dis_max;
  config.corner_thre_ = options.corner_thre;
  config.descriptor_near_num_ = options.descriptor_near_num;
  config.descriptor_min_len_ = options.descriptor_min_len;
  config.descriptor_max_len_ = options.descriptor_max_len;
  config.non_max_suppression_radius_ = options.non_max_suppression_radius;
  config.std_side_resolution_ = options.std_side_resolution;
  config.candidate_num_ = options.candidate_num;
  config.vertex_diff_threshold_ = options.vertex_diff_threshold;
  config.rough_dis_threshold_ = options.rough_dis_threshold;
  config.normal_threshold_ = options.normal_threshold;
  config.dis_threshold_ = options.dis_threshold;
  config.icp_threshold_ = options.icp_threshold;
  return std_descriptor::STDescManager(config);
}

}  // namespace

StdDescriptor::StdDescriptor(Options options)
    : descriptor_manager_(MakeDescriptorManager(options)) {}

std::vector<LoopDescriptor::Match> StdDescriptor::DetectAndAdd(
    const CloudConstPtr& cloud, const SE3& local_from_query,
    std::uint64_t keyframe_id, std::uint64_t reference_id_limit) {
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  input->reserve(cloud->size());
  for (const auto& point : *cloud) {
    pcl::PointXYZI converted;
    // Stable voxel and projection axes follow the fixed LIO world, as upstream.
    converted.getVector3fMap() =
        (local_from_query * point.getVector3fMap().cast<double>()).cast<float>();
    converted.intensity = point.intensity;
    input->push_back(converted);
  }

  std::vector<std_descriptor::STDesc> descriptors;
  descriptor_manager_.GenerateSTDescs(input, descriptors);
  const auto eligible_end = std::lower_bound(
      keyframe_ids_.begin(), keyframe_ids_.end(), reference_id_limit);
  // STD excludes recent dense IDs with a strict difference. q - K converts
  // the caller's eligible actual-ID prefix into that native condition.
  descriptor_manager_.config_setting_.skip_near_num_ =
      keyframe_ids_.end() - eligible_end;
  std::pair<int, double> result{-1, 0.0};
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> local_correction;
  std::vector<std::pair<std_descriptor::STDesc, std_descriptor::STDesc>> pairs;
  descriptor_manager_.SearchLoop(descriptors, result, local_correction, pairs);

  // Advance every epoch, including empty features, alongside native plane IDs.
  descriptor_manager_.AddSTDescs(descriptors);
  keyframe_ids_.push_back(keyframe_id);
  local_poses_.push_back(local_from_query);
  if (result.first < 0) {
    return {};
  }
  const auto reference_id = keyframe_ids_.at(result.first);
  LOG(INFO) << "STD loop candidate: current=" << keyframe_id
            << ", reference=" << reference_id << ", score=" << result.second
            << ", matched triangles=" << pairs.size();
  const SE3 reference_from_query =
      local_poses_.at(result.first).inverse() *
      SE3(SO3(local_correction.second), local_correction.first) * local_from_query;
  return {{reference_id, reference_from_query}};
}

}  // namespace lightning
