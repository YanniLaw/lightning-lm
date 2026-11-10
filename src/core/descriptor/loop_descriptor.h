// Copyright 2026

#ifndef SRC_CORE_DESCRIPTOR_LOOP_DESCRIPTOR_H_
#define SRC_CORE_DESCRIPTOR_LOOP_DESCRIPTOR_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

// A descriptor index used by the serial backend queue.
class LoopDescriptor {
 public:
  struct Match {
    std::uint64_t reference_id = 0;
    // Query IMU coordinates to reference IMU coordinates, when available.
    std::optional<SE3> reference_from_query;
  };

  virtual ~LoopDescriptor() = default;

  // Query history before indexing cloud. IDs increase monotonically; only
  // references with ID < reference_id_limit are eligible. Zero means index only.
  // Cloud holds per-frame samples accumulated in the query's actual IMU frame.
  // local_from_query is its fixed LIO pose, independent of graph optimization.
  virtual std::vector<Match> DetectAndAdd(
      const CloudConstPtr& cloud, const SE3& local_from_query,
      std::uint64_t keyframe_id, std::uint64_t reference_id_limit) = 0;
};

}  // namespace lightning

#endif  // SRC_CORE_DESCRIPTOR_LOOP_DESCRIPTOR_H_
