// Copyright 2026

#ifndef SRC_CORE_DESCRIPTOR_SCAN_CONTEXT_DESCRIPTOR_H_
#define SRC_CORE_DESCRIPTOR_SCAN_CONTEXT_DESCRIPTOR_H_

#include <cstdint>
#include <vector>

#include "core/descriptor/loop_descriptor.h"
#include "core/descriptor/scan_context/Scancontext.h"

namespace lightning {

// Adapts Scan Context to the backend descriptor interface.
class ScanContextDescriptor final : public LoopDescriptor {
 public:
  using Options = scan_context::SCManager::Options;

  explicit ScanContextDescriptor(Options options);

  std::vector<Match> DetectAndAdd(const CloudConstPtr& cloud,
                                  const SE3& local_from_query,
                                  std::uint64_t keyframe_id,
                                  std::uint64_t reference_id_limit) override;

 private:
  scan_context::SCManager descriptor_manager_;
  std::vector<std::uint64_t> keyframe_ids_;
  std::vector<SO3> level_from_imu_rotations_;
};

}  // namespace lightning

#endif  // SRC_CORE_DESCRIPTOR_SCAN_CONTEXT_DESCRIPTOR_H_
