// Copyright 2026
//
// Immutable input captured for one loop-matching task.

#ifndef SRC_CORE_LOOP_CLOSING_LOOP_MATCH_DATA_H_
#define SRC_CORE_LOOP_CLOSING_LOOP_MATCH_DATA_H_

#include "common/eigen_types.h"
#include "common/loop_candidate.h"
#include "common/point_def.h"

namespace lightning {

struct LoopMatchInput {
    LoopCandidate candidate;
    CloudPtr target_cloud;
    CloudPtr source_cloud;
    SE3 reference_world_pose;
    Mat4f initial_source_world_pose = Mat4f::Identity();
};

}  // namespace lightning

#endif  // SRC_CORE_LOOP_CLOSING_LOOP_MATCH_DATA_H_
