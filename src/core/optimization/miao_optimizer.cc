// Copyright 2026
//
// miao adapter for the backend graph optimizer interface.

#include "core/optimization/miao_optimizer.h"

#include <glog/logging.h>

#include <cmath>
#include <limits>
#include <utility>

#include "common/std_types.h"
#include "core/miao/core/graph/optimizer.h"
#include "core/miao/core/opti_algo/algo_select.h"
#include "core/miao/core/robust_kernel/cauchy.h"
#include "core/miao/core/types/edge_se3.h"
#include "core/miao/core/types/edge_se3_height_prior.h"
#include "core/miao/core/types/vertex_se3.h"

namespace lightning {

namespace {

bool IsFinite(double value) { return std::isfinite(value); }

}  // namespace

MiaoOptimizer::MiaoOptimizer() {
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);
}

MiaoOptimizer::~MiaoOptimizer() = default;

bool MiaoOptimizer::IsValidPose(const SE3& pose) const { return pose.matrix().allFinite(); }

bool MiaoOptimizer::ToMiaoId(std::uint64_t id, int& miao_id) const {
    if (id > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        LOG(ERROR) << "pose graph id exceeds miao integer range: " << id;
        return false;
    }
    miao_id = static_cast<int>(id);
    return true;
}

std::shared_ptr<miao::VertexSE3> MiaoOptimizer::FindVertex(std::uint64_t id) const {
    const auto iter = vertices_.find(id);
    if (iter == vertices_.end()) {
        return nullptr;
    }
    return iter->second;
}

bool MiaoOptimizer::AddNode(const GraphNode& node) {
    if (!optimizer_ || !IsValidPose(node.pose) || vertices_.count(node.id) != 0U) {
        LOG(ERROR) << "cannot add invalid or duplicate pose graph node: " << node.id;
        return false;
    }

    int miao_id = 0;
    if (!ToMiaoId(node.id, miao_id)) {
        return false;
    }
    // miao incremental mode does not support fixed vertices. The current
    // PoseGraph deliberately keeps every vertex unfixed to preserve behavior.
    if (node.fixed) {
        LOG(ERROR) << "fixed nodes are not supported by the incremental miao adapter";
        return false;
    }

    auto vertex = std::make_shared<miao::VertexSE3>();
    vertex->SetId(miao_id);
    vertex->SetEstimate(node.pose);
    if (!optimizer_->AddVertex(vertex)) {
        LOG(ERROR) << "miao rejected pose graph node: " << node.id;
        return false;
    }
    vertices_.emplace(node.id, std::move(vertex));
    return true;
}

bool MiaoOptimizer::AddRelativeConstraint(const RelativeConstraint& constraint) {
    if (!optimizer_ || constraint.id == 0U || !IsValidPose(constraint.measurement) ||
        !constraint.information.allFinite()) {
        LOG(ERROR) << "cannot add invalid relative constraint: " << constraint.id;
        return false;
    }
    if (constraint.is_loop &&
        (!IsFinite(constraint.robust_delta) || constraint.robust_delta <= 0.0)) {
        LOG(ERROR) << "loop constraint has invalid robust-kernel delta: " << constraint.id;
        return false;
    }
    if (constraint_ids_.count(constraint.id) != 0U) {
        LOG(ERROR) << "duplicate relative constraint: " << constraint.id;
        return false;
    }

    const auto from = FindVertex(constraint.from_id);
    const auto to = FindVertex(constraint.to_id);
    if (!from || !to) {
        LOG(ERROR) << "relative constraint references a missing node: " << constraint.id;
        return false;
    }

    auto edge = std::make_shared<miao::EdgeSE3>();
    edge->SetVertex(0, from);
    edge->SetVertex(1, to);
    edge->SetMeasurement(constraint.measurement);
    edge->SetInformation(constraint.information);
    if (constraint.is_loop) {
        auto robust_kernel = std::make_shared<miao::RobustKernelCauchy>();
        robust_kernel->SetDelta(constraint.robust_delta);
        edge->SetRobustKernel(std::move(robust_kernel));
    }
    if (!optimizer_->AddEdge(edge)) {
        LOG(ERROR) << "miao rejected relative constraint: " << constraint.id;
        return false;
    }
    constraint_ids_.insert(constraint.id);
    if (constraint.is_loop) {
        loop_edges_.emplace(constraint.id, LoopEdge{constraint.id, std::move(edge)});
    }
    return true;
}

bool MiaoOptimizer::AddHeightConstraint(const HeightConstraint& constraint) {
    if (!optimizer_ || constraint.id == 0U || !IsFinite(constraint.height) ||
        !IsFinite(constraint.information) || constraint.information <= 0.0) {
        LOG(ERROR) << "cannot add invalid height constraint: " << constraint.id;
        return false;
    }
    if (constraint_ids_.count(constraint.id) != 0U) {
        LOG(ERROR) << "duplicate height constraint: " << constraint.id;
        return false;
    }
    const auto vertex = FindVertex(constraint.node_id);
    if (!vertex) {
        LOG(ERROR) << "height constraint references a missing node: " << constraint.node_id;
        return false;
    }

    auto edge = std::make_shared<miao::EdgeHeightPrior>();
    edge->SetVertex(0, vertex);
    edge->SetMeasurement(constraint.height);
    edge->SetInformation(Mat1d::Identity() * constraint.information);
    if (!optimizer_->AddEdge(edge)) {
        LOG(ERROR) << "miao rejected height constraint: " << constraint.id;
        return false;
    }
    constraint_ids_.insert(constraint.id);
    return true;
}

OptimizationResult MiaoOptimizer::Optimize(int max_iterations) {
    OptimizationResult result;
    if (!optimizer_ || max_iterations <= 0 || optimizer_->GetEdges().empty()) {
        return result;
    }

    std::unordered_map<std::uint64_t, SE3> estimates;
    estimates.reserve(vertices_.size());
    for (const auto& [id, vertex] : vertices_) {
        estimates.emplace(id, vertex->Estimate());
    }

    if (!optimizer_->InitializeOptimization()) {
        LOG(ERROR) << "miao failed to initialize pose graph optimization";
        return result;
    }
    optimizer_->SetVerbose(false);
    const int iterations = optimizer_->Optimize(max_iterations);
    if (iterations <= 0) {
        for (const auto& [id, pose] : estimates) {
            vertices_.at(id)->SetEstimate(pose);
        }
        LOG(ERROR) << "miao failed to optimize pose graph";
        return result;
    }

    result.usable = true;
    result.iterations = iterations;
    result.nodes.reserve(vertices_.size());
    for (const auto& [id, vertex] : vertices_) {
        if (!IsValidPose(vertex->Estimate())) {
            LOG(ERROR) << "miao produced a non-finite pose for node: " << id;
            for (const auto& [restore_id, pose] : estimates) {
                vertices_.at(restore_id)->SetEstimate(pose);
            }
            return OptimizationResult();
        }
        result.nodes.push_back(NodeEstimate{id, vertex->Estimate()});
    }

    result.loops.reserve(loop_edges_.size());
    for (auto& [id, loop_edge] : loop_edges_) {
        const auto& edge = loop_edge.edge;
        const double chi2 = edge->Chi2();
        if (!IsFinite(chi2)) {
            LOG(ERROR) << "miao produced a non-finite loop residual: " << id;
            for (const auto& [restore_id, pose] : estimates) {
                vertices_.at(restore_id)->SetEstimate(pose);
            }
            return OptimizationResult();
        }
        const auto robust_kernel = edge->GetRobustKernel();
        bool is_outlier = false;
        if (robust_kernel) {
            is_outlier = chi2 > robust_kernel->Delta();
            if (is_outlier) {
                edge->SetLevel(1);
            } else {
                edge->SetRobustKernel(nullptr);
            }
        }
        result.loops.push_back(LoopConstraintState{id, is_outlier, chi2});
    }
    return result;
}

}  // namespace lightning
