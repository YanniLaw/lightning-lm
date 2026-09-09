// Copyright 2026
//
// Pose graph maintenance and optimization.

#include "core/loop_closing/pose_graph.h"

#include "core/miao/core/graph/optimizer.h"
#include "core/miao/core/opti_algo/algo_select.h"
#include "core/miao/core/robust_kernel/cauchy.h"
#include "core/miao/core/types/edge_se3.h"
#include "core/miao/core/types/edge_se3_height_prior.h"
#include "core/miao/core/types/vertex_se3.h"
#include "io/yaml_io.h"

#include <glog/logging.h>

#include <utility>

namespace lightning {

PoseGraph::PoseGraph(Options options) : options_(std::move(options)) {}

PoseGraph::~PoseGraph() { Stop(); }

void PoseGraph::Init(const std::string& yaml_path) {
    if (initialized_) {
        return;
    }

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);
        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
    }

    LoopClosing::Options loop_options;
    loop_options.verbose_ = options_.verbose_;
    loop_options.loop_kf_gap_ = options_.loop_kf_gap_;
    loop_options.min_id_interval_ = options_.min_id_interval_;
    loop_options.closest_id_th_ = options_.closest_id_th_;
    loop_options.max_range_ = options_.max_range_;
    loop_options.ndt_score_th_ = options_.ndt_score_th_;
    loop_closing_.SetOptions(loop_options);

    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    initialized_ = true;
    accepting_.store(true);

    if (options_.online_mode_) {
        LOG(INFO) << "pose graph is running in online mode";
        keyframe_thread_.SetProcFunc([this](const Keyframe::Ptr& keyframe) { HandleKeyframe(keyframe); });
        keyframe_thread_.SetName("handle loop closure");
        keyframe_thread_.Start();
    }
}

void PoseGraph::AddKeyframe(Keyframe::Ptr keyframe) {
    if (!accepting_.load() || !keyframe) {
        return;
    }

    if (options_.online_mode_) {
        keyframe_thread_.AddMessage(keyframe);
    } else {
        HandleKeyframe(keyframe);
    }
}

void PoseGraph::HandleKeyframe(const Keyframe::Ptr& keyframe) {
    if (!keyframe || keyframe == last_keyframe_ || !initialized_) {
        return;
    }

    keyframes_.Add(keyframe);

    const auto constraints = loop_closing_.ComputeConstraints(keyframe, keyframes_);
    const bool optimized = UpdateGraphAndOptimize(keyframe, constraints);
    if (optimized && optimized_callback_) {
        optimized_callback_();
    }
    if (optimized) {
        LOG(INFO) << "optimize finished, loops: " << loop_edges_.size();
    }

    last_keyframe_ = keyframe;
}

void PoseGraph::Stop() {
    accepting_.store(false);
    if (options_.online_mode_) {
        keyframe_thread_.Quit();
    }
}

bool PoseGraph::UpdateGraphAndOptimize(const Keyframe::Ptr& current,
                                       const std::vector<LoopCandidate>& constraints) {
    if (!current || !optimizer_) {
        return false;
    }

    auto vertex = std::make_shared<miao::VertexSE3>();
    vertex->SetId(current->GetID());
    vertex->SetEstimate(current->GetOptPose());

    optimizer_->AddVertex(vertex);
    keyframe_vertices_.emplace_back(vertex);

    // Add motion constraints to up to two preceding keyframes.
    /// 上一个关键帧的运动约束
    const std::size_t current_index = keyframes_.FindIndex(current->GetID());
    const auto& all_keyframes = keyframes_.GetAll();
    for (std::size_t i = 1; i < 3 && i <= current_index; ++i) {
        const auto& last_keyframe = all_keyframes[current_index - i];
        auto last_vertex = optimizer_->GetVertex(last_keyframe->GetID());
        if (last_vertex == nullptr) {
            continue;
        }

        auto edge = std::make_shared<miao::EdgeSE3>();
        edge->SetVertex(0, last_vertex);
        edge->SetVertex(1, vertex);

        const SE3 motion = last_keyframe->GetLIOPose().inverse() * current->GetLIOPose();
        edge->SetMeasurement(motion);
        edge->SetInformation(info_motion_);
        optimizer_->AddEdge(edge);
    }
    /// 高度约束
    if (options_.with_height_) {
        auto edge = std::make_shared<miao::EdgeHeightPrior>();
        edge->SetVertex(0, vertex);
        edge->SetMeasurement(0);
        edge->SetInformation(Mat1d::Identity() * 1.0 /
                             (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(edge);
    }
    /// 回环的约束
    for (const auto& constraint : constraints) {
        auto vertex1 = optimizer_->GetVertex(constraint.idx1_);
        auto vertex2 = optimizer_->GetVertex(constraint.idx2_);
        if (vertex1 == nullptr || vertex2 == nullptr) {
            LOG(WARNING) << "skip loop constraint with missing vertex: " << constraint.idx1_ << ", "
                         << constraint.idx2_;
            continue;
        }

        auto edge = std::make_shared<miao::EdgeSE3>();
        edge->SetVertex(0, vertex1);
        edge->SetVertex(1, vertex2);
        edge->SetMeasurement(constraint.Tij_);
        edge->SetInformation(info_loops_);

        auto robust_kernel = std::make_shared<miao::RobustKernelCauchy>();
        robust_kernel->SetDelta(options_.rk_loop_th_);
        edge->SetRobustKernel(robust_kernel);

        optimizer_->AddEdge(edge);
        loop_edges_.emplace_back(edge);
    }

    if (optimizer_->GetEdges().empty() || constraints.empty()) {
        return false;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(false);
    optimizer_->Optimize(20);
    /// remove outliers
    int outlier_count = 0;
    for (const auto& edge : loop_edges_) {
        if (edge->GetRobustKernel() == nullptr) {
            continue;
        }

        if (edge->Chi2() > edge->GetRobustKernel()->Delta()) {
            edge->SetLevel(1);
            ++outlier_count;
        } else {
            edge->SetRobustKernel(nullptr);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << outlier_count << "/" << loop_edges_.size();
    }

    for (const auto& graph_vertex : keyframe_vertices_) {
        const SE3 pose = graph_vertex->Estimate();
        const auto keyframe = keyframes_.Find(graph_vertex->GetId());
        if (keyframe != nullptr) {
            keyframe->SetOptPose(pose);
        }
    }

    return true;
}

}  // namespace lightning
