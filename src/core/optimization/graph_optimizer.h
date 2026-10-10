// Copyright 2026
//
// Backend graph optimizer interface.

#ifndef SRC_CORE_OPTIMIZATION_GRAPH_OPTIMIZER_H_
#define SRC_CORE_OPTIMIZATION_GRAPH_OPTIMIZER_H_

#include <cstdint>
#include <vector>

#include "common/eigen_types.h"

namespace lightning {

struct GraphNode {
    std::uint64_t id = 0;
    SE3 pose;
    bool fixed = false;
};

struct RelativeConstraint {
    std::uint64_t id = 0;
    std::uint64_t from_id = 0;
    std::uint64_t to_id = 0;
    SE3 measurement;
    Mat6d information = Mat6d::Identity();
    bool is_loop = false;
    double robust_delta = 0.0;
};

struct HeightConstraint {
    std::uint64_t id = 0;
    std::uint64_t node_id = 0;
    double height = 0.0;
    double information = 0.0;
};

struct NodeEstimate {
    std::uint64_t id = 0;
    SE3 pose;
};

struct LoopConstraintState {
    std::uint64_t id = 0;
    bool is_outlier = false;
    double chi2 = 0.0;
};

struct OptimizationResult {
    bool usable = false;
    int iterations = 0;
    std::vector<NodeEstimate> nodes;
    std::vector<LoopConstraintState> loops;
};

class GraphOptimizer {
   public:
    virtual ~GraphOptimizer() = default;

    virtual bool AddNode(const GraphNode& node) = 0;
    virtual bool AddRelativeConstraint(const RelativeConstraint& constraint) = 0;
    virtual bool AddHeightConstraint(const HeightConstraint& constraint) = 0;
    virtual OptimizationResult Optimize(int max_iterations) = 0;
};

}  // namespace lightning

#endif  // SRC_CORE_OPTIMIZATION_GRAPH_OPTIMIZER_H_
