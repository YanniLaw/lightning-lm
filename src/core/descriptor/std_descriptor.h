// Copyright 2026

#ifndef SRC_CORE_DESCRIPTOR_STD_DESCRIPTOR_H_
#define SRC_CORE_DESCRIPTOR_STD_DESCRIPTOR_H_

#include <cstdint>
#include <vector>

#include "core/descriptor/loop_descriptor.h"
#include "core/descriptor/std/STDesc.h"

namespace lightning {

// Adapts the official STD core to the backend descriptor interface.
class StdDescriptor final : public LoopDescriptor {
 public:
  struct Options {
    int maximum_corner_num = 500;
    double plane_detection_thre = 0.01;
    double plane_merge_normal_thre = 0.1;
    double voxel_size = 1.0;
    int voxel_init_num = 10;
    double proj_image_resolution = 0.25;
    double proj_dis_min = 0.0;
    double proj_dis_max = 5.0;
    double corner_thre = 10.0;
    int descriptor_near_num = 15;
    double descriptor_min_len = 2.0;
    double descriptor_max_len = 30.0;
    double non_max_suppression_radius = 2.0;
    double std_side_resolution = 0.2;
    int candidate_num = 50;
    double vertex_diff_threshold = 0.8;
    double rough_dis_threshold = 0.015;
    double normal_threshold = 0.2;
    double dis_threshold = 0.5;
    double icp_threshold = 0.2;
  };

  explicit StdDescriptor(Options options);

  StdDescriptor(const StdDescriptor&) = delete;
  StdDescriptor& operator=(const StdDescriptor&) = delete;

  std::vector<Match> DetectAndAdd(const CloudConstPtr& cloud,
                                const SE3& local_from_query,
                                std::uint64_t keyframe_id,
                                std::uint64_t reference_id_limit) override;

 private:
  std_descriptor::STDescManager descriptor_manager_;
  // Native frame indices include epochs without descriptors.
  std::vector<std::uint64_t> keyframe_ids_;
  std::vector<SE3> local_poses_;
};

}  // namespace lightning

#endif  // SRC_CORE_DESCRIPTOR_STD_DESCRIPTOR_H_
