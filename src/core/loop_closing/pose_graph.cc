// Copyright 2026
//
// Pose graph maintenance and optimization.

#include "core/loop_closing/pose_graph.h"

#include <pcl/common/transforms.h>

#include "core/lio/lio_result_conversion.h"

#include "core/miao/core/graph/optimizer.h"
#include "core/miao/core/opti_algo/algo_select.h"
#include "core/miao/core/robust_kernel/cauchy.h"
#include "core/miao/core/types/edge_se3.h"
#include "core/miao/core/types/edge_se3_height_prior.h"
#include "core/miao/core/types/vertex_se3.h"
#include "io/yaml_io.h"

#include <glog/logging.h>

#include <limits>
#include <utility>

namespace lightning {

PoseGraph::PoseGraph(Options options) : options_(std::move(options)) {}

PoseGraph::~PoseGraph() { Stop(); }

void PoseGraph::Init(const std::string& yaml_path) {
    if (initialized_) {
        return;
    }

    if (!yaml_path.empty() && options_.enable_loop_closing_) {
        YAML_IO yaml(yaml_path);
        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
    }

    if (options_.enable_loop_closing_) {
        LoopClosing::Options loop_options;
        loop_options.verbose_ = options_.verbose_;
        loop_options.loop_kf_gap_ = options_.loop_kf_gap_;
        loop_options.min_id_interval_ = options_.min_id_interval_;
        loop_options.closest_id_th_ = options_.closest_id_th_;
        loop_options.max_range_ = options_.max_range_;
        loop_options.ndt_score_th_ = options_.ndt_score_th_;
        loop_closing_.SetOptions(loop_options);
    }

    if (options_.enable_loop_closing_) {
        /// setup miao
        miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                     miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
        config.incremental_mode_ = true;
        optimizer_ = miao::SetupOptimizer<6, 3>(config);
    }

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
        // Accepted backend nodes must not be silently discarded while the
        // optimizer catches up with the frontend.
        keyframe_thread_.SetMaxSize(std::numeric_limits<std::size_t>::max());
        keyframe_thread_.SetProcFunc([this](const Keyframe::Ptr& keyframe) { HandleKeyframe(keyframe); });
        keyframe_thread_.SetName("handle loop closure");
        keyframe_thread_.Start();
    }
}

Keyframe::Ptr PoseGraph::AddKeyframe(const LIOResult& result) {
    if (result.update_type != LIOUpdateType::kScanMatched || !result.keyframe_selected ||
        !result.state.pose_is_valid || !result.cloud || result.cloud->empty() ||
        !result.body_from_lidar.matrix().allFinite()) {
        return nullptr;
    }

    for (const PointType& point : result.cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.intensity)) {
            return nullptr;
        }
    }

    NavState state;
    if (!ConvertToNavState(result.state, state)) {
        return nullptr;
    }

    Keyframe::Ptr keyframe;
    {
        std::lock_guard<std::mutex> lock(accepting_mutex_);
        if (!accepting_.load() || result.state.timestamp <= last_accepted_timestamp_) {
            return nullptr;
        }

        auto cloud = std::make_shared<PointCloudType>();
        pcl::transformPointCloud(*result.cloud, *cloud, result.body_from_lidar.matrix());
        keyframe = std::make_shared<Keyframe>(static_cast<unsigned long>(next_keyframe_id_++), cloud, state);
        accepted_keyframes_.emplace_back(keyframe);
        last_accepted_timestamp_ = result.state.timestamp;

        if (options_.online_mode_) {
            keyframe_thread_.AddMessage(keyframe);
        }
    }

    if (!options_.online_mode_) {
        HandleKeyframe(keyframe);
    }
    return keyframe;
}

LIOInputStatus PoseGraph::AddWheelOdometry(const WheelOdometryData& data) {
    if (data.timestamp_ns <= 0) {
        return LIOInputStatus::kInvalidData;
    }
    if (!accepting_.load()) {
        return LIOInputStatus::kNotRunning;
    }
    return LIOInputStatus::kUnsupported;
}

std::vector<KeyframeMapEntry> PoseGraph::GetMapSnapshot() const {
    // Serialize with backend pose updates so all fields in one snapshot refer
    // to the same optimization state.  Disk and PCL work happens after this
    // lock is released.
    std::lock_guard<std::mutex> backend_lock(backend_mutex_);

    std::vector<Keyframe::Ptr> accepted_keyframes;
    {
        std::lock_guard<std::mutex> lock(accepting_mutex_);
        accepted_keyframes = accepted_keyframes_;
    }

    std::vector<KeyframeMapEntry> snapshot;
    snapshot.reserve(accepted_keyframes.size());
    for (const auto& keyframe : accepted_keyframes) {
        if (!keyframe || !keyframe->GetCloud()) {
            continue;
        }

        KeyframeMapEntry entry;
        entry.id = keyframe->GetID();
        entry.timestamp = keyframe->GetState().timestamp_;
        entry.cloud = std::make_shared<const PointCloudType>(*keyframe->GetCloud());
        entry.lio_pose = keyframe->GetLIOPose();
        entry.optimized_pose = keyframe->GetOptPose();
        snapshot.emplace_back(std::move(entry));
    }
    return snapshot;
}

void PoseGraph::HandleKeyframe(const Keyframe::Ptr& keyframe) {
    bool optimized = false;
    std::size_t loop_count = 0;
    OptimizedCallback optimized_callback;
    DataCallback data_callback;
    PoseGraphDataPtr data_snapshot;
    {
        std::lock_guard<std::mutex> lock(backend_mutex_);
        if (!keyframe || keyframe == last_keyframe_ || !initialized_) {
            return;
        }

        keyframes_.Add(keyframe);

        const auto constraints = options_.enable_loop_closing_
                                     ? loop_closing_.ComputeConstraints(keyframe, keyframes_)
                                     : std::vector<LoopCandidate>();
        optimized = UpdateGraphAndOptimize(keyframe, constraints);
        loop_count = loop_edges_.size();
        last_keyframe_ = keyframe;
        optimized_callback = optimized_callback_;
        data_callback = data_callback_;
        if (data_callback_) {
            data_snapshot = CreateDataSnapshot(optimized);
        }
    }

    // Do not invoke external callbacks while the backend lock is held.  A
    // visualization callback may synchronously request another snapshot.
    if (optimized && optimized_callback) {
        optimized_callback();
    }
    if (optimized) {
        LOG(INFO) << "optimize finished, loops: " << loop_count;
    }

    if (data_callback) {
        data_callback(std::move(data_snapshot));
    }
}

PoseGraphDataPtr PoseGraph::CreateDataSnapshot(bool optimized) {
    auto data = std::make_shared<PoseGraphData>();
    data->version = ++data_version_;
    data->optimized = optimized;

    const auto& all_keyframes = keyframes_.GetAll();
    data->nodes.reserve(all_keyframes.size());
    for (const auto& keyframe : all_keyframes) {
        if (!keyframe) {
            continue;
        }

        PoseGraphNode node;
        node.id = keyframe->GetID();
        node.timestamp = keyframe->GetState().timestamp_;
        node.pose = keyframe->GetOptPose();
        data->nodes.emplace_back(std::move(node));
    }

    data->loop_constraints.reserve(loop_edges_.size());
    for (const auto& edge : loop_edges_) {
        if (!edge || edge->GetVertices().size() < 2 || edge->GetVertex(0) == nullptr ||
            edge->GetVertex(1) == nullptr) {
            continue;
        }

        PoseGraphConstraint constraint;
        constraint.from_id = static_cast<unsigned long>(edge->GetVertex(0)->GetId());
        constraint.to_id = static_cast<unsigned long>(edge->GetVertex(1)->GetId());
        constraint.is_outlier = edge->Level() > 0;
        data->loop_constraints.emplace_back(std::move(constraint));
    }

    return data;
}

void PoseGraph::Stop() {
    {
        std::lock_guard<std::mutex> lock(accepting_mutex_);
        accepting_.store(false);
    }
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
