// Copyright 2026
//
// miao adapter for the backend graph optimizer interface.

#ifndef SRC_CORE_OPTIMIZATION_MIAO_OPTIMIZER_H_
#define SRC_CORE_OPTIMIZATION_MIAO_OPTIMIZER_H_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "core/optimization/graph_optimizer.h"

namespace lightning::miao {
class EdgeSE3;
class Optimizer;
class VertexSE3;
}  // namespace lightning::miao

namespace lightning {

class MiaoOptimizer final : public GraphOptimizer {
   public:
    MiaoOptimizer();
    ~MiaoOptimizer() override;

    bool AddNode(const GraphNode& node) override;
    bool AddRelativeConstraint(const RelativeConstraint& constraint) override;
    bool AddHeightConstraint(const HeightConstraint& constraint) override;
    OptimizationResult Optimize(int max_iterations) override;

   private:
    struct LoopEdge {
        std::uint64_t id = 0;
        std::shared_ptr<miao::EdgeSE3> edge;
    };

    bool IsValidPose(const SE3& pose) const;
    bool ToMiaoId(std::uint64_t id, int& miao_id) const;
    std::shared_ptr<miao::VertexSE3> FindVertex(std::uint64_t id) const;

    std::shared_ptr<miao::Optimizer> optimizer_;
    std::unordered_map<std::uint64_t, std::shared_ptr<miao::VertexSE3>> vertices_;
    std::unordered_map<std::uint64_t, LoopEdge> loop_edges_;
    std::unordered_set<std::uint64_t> constraint_ids_;
};

}  // namespace lightning

#endif  // SRC_CORE_OPTIMIZATION_MIAO_OPTIMIZER_H_
