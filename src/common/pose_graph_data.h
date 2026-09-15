// Copyright 2026
//
// Transport-independent pose graph visualization data.

#ifndef LIGHTNING_POSE_GRAPH_DATA_H
#define LIGHTNING_POSE_GRAPH_DATA_H

#include <cstdint>
#include <memory>
#include <vector>

#include "common/eigen_types.h"

namespace lightning {

/// A pose graph node exposed to visualization or other external consumers.
struct PoseGraphNode {
    unsigned long id = 0;
    double timestamp = 0.0;
    SE3 pose = SE3();
};

/// A loop-closure edge exposed to visualization consumers.
struct PoseGraphConstraint {
    unsigned long from_id = 0;
    unsigned long to_id = 0;
    bool is_outlier = false;
};

/// Immutable snapshot of the current pose graph state.
struct PoseGraphData {
    std::uint64_t version = 0;
    bool optimized = false;
    std::vector<PoseGraphNode> nodes;
    std::vector<PoseGraphConstraint> loop_constraints;
};

using PoseGraphDataPtr = std::shared_ptr<const PoseGraphData>;

}  // namespace lightning

#endif  // LIGHTNING_POSE_GRAPH_DATA_H
