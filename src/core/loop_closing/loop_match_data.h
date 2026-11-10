// Copyright 2026
//
// Immutable input captured for one loop-matching task.

#ifndef SRC_CORE_LOOP_CLOSING_LOOP_MATCH_DATA_H_
#define SRC_CORE_LOOP_CLOSING_LOOP_MATCH_DATA_H_

#include <optional>

#include "common/eigen_types.h"
#include "common/loop_candidate.h"
#include "common/point_def.h"

namespace lightning {

struct LoopMatchInput {
    LoopCandidate candidate;
    CloudConstPtr target_cloud;
    CloudConstPtr source_cloud;
    SE3 reference_world_pose;
    Mat4f initial_source_world_pose = Mat4f::Identity();
    // Optional descriptor guess; geometry checks retain the original baseline.
    std::optional<Mat4f> registration_guess;
};

}  // namespace lightning

#endif  // SRC_CORE_LOOP_CLOSING_LOOP_MATCH_DATA_H_
